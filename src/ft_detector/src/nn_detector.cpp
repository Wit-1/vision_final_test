#include "ft_detector/nn_detector.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <fstream>
#include <stdexcept>
#include <tuple>
#include <utility>

#include <onnxruntime_cxx_api.h>
#include <opencv2/imgproc.hpp>

namespace ft_detector {
namespace {
// 固定模型输出协议；筛选门限和运行线程数仍从配置读取。
constexpr int kChannels = 21, kCandidates = 6300;

// 模型已输出分数，检查合法区间，不再额外执行 sigmoid。
bool unitScore(double value) { return std::isfinite(value) && value >= 0.0 && value <= 1.0; }

// 用四角点的外接矩形近似候选范围，供 NMS 比较重叠。
cv::Rect2f bounds(const ArmorCandidate& armor) {
  float left = armor.corners[0].x, right = left, top = armor.corners[0].y, bottom = top;
  for (const auto& point : armor.corners) {
    left = std::min(left, point.x); right = std::max(right, point.x);
    top = std::min(top, point.y); bottom = std::max(bottom, point.y);
  }
  return {left, top, right - left, bottom - top};
}

// 计算两个外接矩形的交并比。
double iou(const cv::Rect2f& first, const cv::Rect2f& second) {
  const double intersection = (first & second).area();
  return intersection / (first.area() + second.area() - intersection);
}
}  // namespace

// 先减补边，再除实际缩放比例，将模型画布坐标还原到原图。
cv::Point2f LetterboxTransform::toOriginal(const cv::Point2f& point) const {
  return {static_cast<float>((point.x - left) / scale_x),
          static_cast<float>((point.y - top) / scale_y)};
}

// 校验固定输入协议、几何门限、线程数和四项动态阈值。
void NnDetector::validateParams(const NnParams& params) {
  const auto& thresholds = params.thresholds;
  if (params.model_path.empty() || params.input_width != 640 || params.input_height != 480 ||
      params.pad_value < 0 || params.pad_value > 255 || params.cpu_threads <= 0 ||
      params.max_candidates <= 0 || params.max_candidates > kCandidates ||
      !std::isfinite(params.large_armor_min_spacing_ratio) || params.large_armor_min_spacing_ratio <= 0.0 ||
      !std::isfinite(params.min_quad_area_px2) || params.min_quad_area_px2 <= 0.0 ||
      !std::isfinite(params.min_edge_length_px) || params.min_edge_length_px <= 0.0 ||
      !unitScore(thresholds.color_score) || !unitScore(thresholds.number_score) ||
      !unitScore(thresholds.confidence) || !unitScore(thresholds.nms_iou)) {
    throw std::invalid_argument("Invalid neural model protocol, geometry or thresholds");
  }
}

// 将 BGR 原图变为 RGB、归一化的 NCHW 张量，同时保存坐标反变换。
NnInput prepareNnInput(const cv::Mat& bgr, const NnParams& params) {
  if (bgr.empty() || bgr.type() != CV_8UC3)
    throw std::invalid_argument("Neural detector expects a non-empty CV_8UC3 image");
  NnInput input;
  auto& transform = input.transform;
  transform.original_size = bgr.size();
  // 等比例缩放并居中补边，使用取整后的尺寸记录实际横纵比例。
  const double scale = std::min(static_cast<double>(params.input_width) / bgr.cols,
                                static_cast<double>(params.input_height) / bgr.rows);
  transform.resized_size = {std::max(1, static_cast<int>(std::lround(bgr.cols * scale))),
                            std::max(1, static_cast<int>(std::lround(bgr.rows * scale)))};
  transform.left = (params.input_width - transform.resized_size.width) / 2;
  transform.top = (params.input_height - transform.resized_size.height) / 2;
  // Save the actual integer resize factors, including rounding on odd-sized images.
  transform.scale_x = static_cast<double>(transform.resized_size.width) / bgr.cols;
  transform.scale_y = static_cast<double>(transform.resized_size.height) / bgr.rows;
  cv::Mat resized, canvas, rgb, normalized;
  cv::resize(bgr, resized, transform.resized_size, 0.0, 0.0, cv::INTER_LINEAR);
  cv::copyMakeBorder(resized, canvas, transform.top,
      params.input_height - resized.rows - transform.top, transform.left,
      params.input_width - resized.cols - transform.left, cv::BORDER_CONSTANT,
      cv::Scalar::all(params.pad_value));
  // 调整通道顺序和数值范围，再把三层像素写入连续张量。
  cv::cvtColor(canvas, rgb, cv::COLOR_BGR2RGB);
  rgb.convertTo(normalized, CV_32F, 1.0 / 255.0);
  const int shape[] = {1, 3, params.input_height, params.input_width};
  input.blob.create(4, shape, CV_32F);
  std::array<cv::Mat, 3> channels;
  for (int channel = 0; channel < 3; ++channel)
    channels[channel] = cv::Mat(params.input_height, params.input_width, CV_32F,
                                input.blob.ptr<float>(0, channel));
  cv::split(normalized, channels.data());
  return input;
}

// 解码分数与四点，执行几何筛选和 NMS，返回共用装甲板候选。
std::vector<ArmorCandidate> decodeNnOutput(const float* values, std::size_t count,
                        const LetterboxTransform& transform, const NnParams& params) {
  if (!values || count != kChannels * kCandidates || transform.original_size.empty() ||
      !std::isfinite(transform.scale_x) || !std::isfinite(transform.scale_y) ||
      transform.scale_x <= 0.0 || transform.scale_y <= 0.0)
    throw std::invalid_argument("Unexpected neural output shape or resize transform");
  // 输出按通道连续存储，通过 channel×6300+candidate 访问。
  const auto score = [values](int channel, int candidate) { return values[channel * kCandidates + candidate]; };
  std::vector<ArmorCandidate> armors;
  for (int candidate = 0; candidate < kCandidates; ++candidate) {
    bool valid_scores = true;
    for (int channel = 0; channel < 13; ++channel) {
      const float value = score(channel, candidate);
      if (!unitScore(value)) { valid_scores = false; break; }
    }
    if (!valid_scores) continue;
    // 颜色和编号分别取最高分；熄灭或其他颜色胜出时直接拒绝。
    int color = 0, number = 0;
    for (int channel = 1; channel < 4; ++channel)
      if (score(channel, candidate) > score(color, candidate)) color = channel;
    for (int index = 1; index < 9; ++index)
      if (score(index + 4, candidate) > score(number + 4, candidate)) number = index;
    const double color_score = score(color, candidate), number_score = score(number + 4, candidate);
    const double confidence = color_score * number_score;
    if (color > 1 || color_score < params.thresholds.color_score ||
        number_score < params.thresholds.number_score || confidence < params.thresholds.confidence) continue;
    // 重排为共用点序，并在反变换后检查原图边界。
    ArmorCandidate armor;
    armor.id = candidate;  // Deterministic tie-break only; replaced by frame-local IDs after NMS.
    armor.is_red = color == 1;
    armor.number_class = number;
    armor.confidence = static_cast<float>(confidence);
    constexpr int order[] = {0, 3, 2, 1};  // Model TL/BL/BR/TR -> shared TL/TR/BR/BL.
    bool valid_points = true;
    for (int point = 0; point < 4; ++point) {
      const float x = score(13 + 2 * order[point], candidate), y = score(14 + 2 * order[point], candidate);
      if (!std::isfinite(x) || !std::isfinite(y)) { valid_points = false; break; }
      armor.corners[point] = transform.toOriginal({x, y});
      const auto& p = armor.corners[point];
      if (!std::isfinite(p.x) || !std::isfinite(p.y) || p.x < 0.0F || p.y < 0.0F ||
          p.x >= transform.original_size.width || p.y >= transform.original_size.height) valid_points = false;
    }
    if (!valid_points) continue;
    // 去掉非凸、过小或边长不足的四边形。
    const std::vector<cv::Point2f> points(armor.corners.begin(), armor.corners.end());
    if (!cv::isContourConvex(points) || cv::contourArea(points, true) < params.min_quad_area_px2) continue;
    for (int point = 0; point < 4; ++point)
      if (cv::norm(armor.corners[point] - armor.corners[(point + 1) % 4]) < params.min_edge_length_px)
        valid_points = false;
    if (!valid_points) continue;
    // 模型没有大小输出，沿用灯条间距与长度之比判别板型。
    const auto left = (armor.corners[0] + armor.corners[3]) * 0.5F;
    const auto right = (armor.corners[1] + armor.corners[2]) * 0.5F;
    const double mean_length = (cv::norm(armor.corners[0] - armor.corners[3]) +
                                cv::norm(armor.corners[1] - armor.corners[2])) * 0.5;
    armor.is_large_armor = cv::norm(left - right) / mean_length >= params.large_armor_min_spacing_ratio;
    armor.center = (left + right) * 0.5F;
    armors.push_back(armor);
  }
  // 按置信度排序并限制数量，再保留不与高分项过度重叠的候选。
  std::sort(armors.begin(), armors.end(), [](const ArmorCandidate& a, const ArmorCandidate& b) {
    return a.confidence != b.confidence ? a.confidence > b.confidence : a.id < b.id;
  });
  if (armors.size() > static_cast<std::size_t>(params.max_candidates))
    armors.resize(params.max_candidates);
  std::vector<ArmorCandidate> kept;
  for (const auto& candidate : armors) {
    const auto box = bounds(candidate);
    const bool overlap = std::any_of(kept.begin(), kept.end(), [&](const ArmorCandidate& prior) {
      return iou(box, bounds(prior)) > params.thresholds.nms_iou;
    });
    if (!overlap) kept.push_back(candidate);
  }
  // 最终按原图位置排序，重新分配仅在本帧有效的 ID。
  std::sort(kept.begin(), kept.end(), [](const ArmorCandidate& a, const ArmorCandidate& b) {
    return std::tie(a.center.x, a.center.y) < std::tie(b.center.x, b.center.y);
  });
  for (std::size_t index = 0; index < kept.size(); ++index) kept[index].id = index;
  return kept;
}

// 将 ONNX Runtime 资源封装在实现文件中，头文件只暴露检测接口。
struct NnDetector::Impl {
  Ort::Env environment{ORT_LOGGING_LEVEL_WARNING, "ft_nn"};
  Ort::SessionOptions options;
  Ort::Session session{nullptr};
  Ort::MemoryInfo memory = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);

  // 启动时只创建一次 Session，并核对输入输出名称、类型与形状。
  explicit Impl(const NnParams& params) {
    options.SetIntraOpNumThreads(params.cpu_threads);
    options.SetInterOpNumThreads(1);
    options.SetExecutionMode(ExecutionMode::ORT_SEQUENTIAL);
    options.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
    session = Ort::Session(environment, params.model_path.c_str(), options);
    // 不兼容的模型在启动阶段报错，避免逐帧按错误协议读取内存。
    if (session.GetInputCount() != 1 || session.GetOutputCount() != 1)
      throw std::invalid_argument("Expected one model input and one output");
    Ort::AllocatorWithDefaultOptions allocator;
    const auto input_name = session.GetInputNameAllocated(0, allocator);
    const auto output_name = session.GetOutputNameAllocated(0, allocator);
    const auto input_type = session.GetInputTypeInfo(0);
    const auto output_type = session.GetOutputTypeInfo(0);
    const auto input = input_type.GetTensorTypeAndShapeInfo();
    const auto output = output_type.GetTensorTypeAndShapeInfo();
    if (std::string(input_name.get()) != "images" || std::string(output_name.get()) != "output0" ||
        input.GetElementType() != ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT ||
        output.GetElementType() != ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT ||
        input.GetShape() != std::vector<int64_t>({1, 3, params.input_height, params.input_width}) ||
        output.GetShape() != std::vector<int64_t>({1, kChannels, kCandidates}))
      throw std::invalid_argument("Model names, float32 type or NCHW/output shape differ from protocol");
  }
};

// 检查参数及模型文件，再建立长期复用的推理资源。
NnDetector::NnDetector(const NnParams& params) : params_(params) {
  validateParams(params_);
  if (!std::ifstream(params_.model_path).good())
    throw std::invalid_argument("Model not found; run ./run.sh prepare-model: " + params_.model_path);
  impl_.reset(new Impl(params_));
}

// 在 Impl 类型完整的实现文件中释放推理资源。
NnDetector::~NnDetector() = default;

// 校验并更新分数、去重与大小板门限，无需重新加载模型。
void NnDetector::setThresholds(const NnThresholds& thresholds, double large_armor_min_spacing_ratio) {
  auto next = params_;
  next.thresholds = thresholds;
  next.large_armor_min_spacing_ratio = large_armor_min_spacing_ratio;
  validateParams(next);
  params_ = next;
}

// 单帧推理入口：预处理、运行 Session、检查输出并解码候选。
std::vector<ArmorCandidate> NnDetector::detect(const cv::Mat& bgr) {
  auto input = prepareNnInput(bgr, params_);
  const std::array<int64_t, 4> shape{{1, 3, params_.input_height, params_.input_width}};
  auto tensor = Ort::Value::CreateTensor<float>(impl_->memory, input.blob.ptr<float>(), input.blob.total(),
                                               shape.data(), shape.size());
  const char* names_in[] = {"images"};
  const char* names_out[] = {"output0"};
  // 张量借用本帧 blob 内存，该内存在同步推理返回前保持有效。
  auto outputs = impl_->session.Run(Ort::RunOptions{nullptr}, names_in, &tensor, 1, names_out, 1);
  const auto info = outputs.front().GetTensorTypeAndShapeInfo();
  if (info.GetElementType() != ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT ||
      info.GetShape() != std::vector<int64_t>({1, kChannels, kCandidates}))
    throw std::runtime_error("Runtime output differs from model protocol");
  return decodeNnOutput(outputs.front().GetTensorData<float>(), info.GetElementCount(), input.transform, params_);
}

// 向日志和独立工具提供实际使用的推理后端版本。
std::string NnDetector::backendVersion() { return OrtGetApiBase()->GetVersionString(); }

}  // namespace ft_detector
