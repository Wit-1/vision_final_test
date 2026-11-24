#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <stdexcept>
#include <string>
#include <vector>
#include <onnxruntime_cxx_api.h>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <yaml-cpp/yaml.h>
#include <ft_detector/nn_detector.hpp>
#include <ft_detector/pnp_solver.hpp>

namespace fs=std::filesystem;
using Clock=std::chrono::steady_clock;
using namespace ft_detector;
namespace {
struct Options {
  fs::path project,images,reference;
  int threads=2,opencv=2,warmup=20,iterations=120;
  bool write=false;
};

// 工具只调整线程，输入、模型、筛选和板型策略保持相同，ROI 始终关闭。
Options options(int argc,char**argv) {
  Options o;
  for (int i=1;i<argc;++i) {
    std::string key=argv[i];
    if (key=="--write-reference") {o.write=true;continue;}
    if (i+1==argc) throw std::invalid_argument("Missing value: "+key);
    std::string value=argv[++i];
    if (key=="--project") o.project=value;
    else if (key=="--images") o.images=value;
    else if (key=="--reference") o.reference=value;
    else if (key=="--threads") o.threads=std::stoi(value);
    else if (key=="--opencv") o.opencv=std::stoi(value);
    else if (key=="--warmup") o.warmup=std::stoi(value);
    else if (key=="--iterations") o.iterations=std::stoi(value);
    else throw std::invalid_argument("Unknown option: "+key);
  }
  if (!fs::is_directory(o.project) || !fs::is_directory(o.images) || o.reference.empty() ||
      o.threads<1 || o.opencv<1 || o.warmup<1 || o.iterations<1)
    throw std::invalid_argument("Expected project/images/reference, positive thread and sample counts");
  return o;
}

// 使用工程实际的模型、阈值、尺寸和 PnP 条件，避免单独维护一套测试门限。
std::pair<NnParams,PnpParams> parameters(const Options& o) {
  const auto config=o.project/"src/ft_bringup/config";
  const auto model=YAML::LoadFile((config/"model.yaml").string());
  const auto detector=YAML::LoadFile((config/"detector.yaml").string());
  const auto sizes=YAML::LoadFile((config/"armor_dimensions.yaml").string())["pnp"];
  const auto n=model["nn"],t=model["tuning"],p=detector["pnp"];
  NnParams nn;
  nn.model_path=n["model_path"].as<std::string>();
  const std::string token="$(find ft_bringup)";
  const auto found=nn.model_path.find(token);
  if (found!=std::string::npos) nn.model_path.replace(found,token.size(),(o.project/"src/ft_bringup").string());
  nn.input_width=n["input_width"].as<int>(); nn.input_height=n["input_height"].as<int>();
  nn.pad_value=n["pad_value"].as<int>(); nn.cpu_threads=o.threads;
  nn.max_candidates=n["max_candidates"].as<int>();
  nn.large_armor_min_spacing_ratio=detector["traditional"]["large_armor_min_spacing_ratio"].as<double>();
  nn.min_quad_area_px2=p["min_quad_area_px2"].as<double>();
  nn.min_edge_length_px=p["min_edge_length_px"].as<double>();
  nn.thresholds={t["nn_min_color_score"].as<double>(),t["nn_min_number_score"].as<double>(),
    t["nn_min_confidence"].as<double>(),t["nn_nms_iou"].as<double>()};
  const auto size=n["number_size"];
  nn.size_policy.enabled=size["enabled"].as<bool>();
  nn.size_policy.min_score=size["min_score"].as<double>();
  nn.size_policy.min_margin=size["min_margin"].as<double>();
  const auto mapping=size["class_sizes"].as<std::vector<int>>();
  if (mapping.size()!=9) throw std::invalid_argument("Invalid number-size mapping");
  std::copy(mapping.begin(),mapping.end(),nn.size_policy.class_sizes.begin());
  NnDetector::validateParams(nn);
  PnpParams pose;
  const auto dimensions=[&](const std::string& name){
    return ArmorDimensions{sizes[name]["width_m"].as<double>(),sizes[name]["height_m"].as<double>()};
  };
  pose.small=dimensions("small"); pose.large=dimensions("large");
  pose.ambiguity_error_margin_px=p["continuity"]["error_margin_px"].as<double>();
  pose.min_depth_m=p["min_depth_m"].as<double>(); pose.max_depth_m=p["max_depth_m"].as<double>();
  pose.max_reprojection_error_px=p["max_reprojection_error_px"].as<double>();
  pose.min_quad_area_px2=nn.min_quad_area_px2; pose.min_edge_length_px=nn.min_edge_length_px;
  pose.axis_length_m=p["axis_length_m"].as<double>();
  return {nn,pose};
}

struct Sample {std::string name;cv::Mat image;CameraCalibration camera;std::vector<float> reference;};

// JPEG、标定和参考张量提前读入内存，计时不包含磁盘访问。
std::vector<Sample> samples(const Options& o) {
  std::vector<fs::path> paths;
  for (const auto& entry:fs::directory_iterator(o.images)) if (entry.path().extension()==".jpg") paths.push_back(entry.path());
  std::sort(paths.begin(),paths.end());
  const auto cameras=YAML::LoadFile((o.images/"cameras.yaml").string());
  std::vector<Sample> loaded;
  for (const auto& path:paths) {
    Sample s; s.name=path.filename().string();s.image=cv::imread(path.string());
    if (s.image.empty()) throw std::runtime_error("Cannot decode "+path.string());
    const auto camera=cameras[s.name];
    auto k=camera["K"].as<std::vector<double>>();
    if(k.size()!=9)throw std::runtime_error("Invalid K");
    std::copy(k.begin(),k.end(),s.camera.camera_matrix.val);
    s.camera.distortion=camera["D"].as<std::vector<double>>();
    s.camera.image_size=s.image.size(); s.camera.distortion_model=camera["model"].as<std::string>();
    if (!PnpSolver::validCalibration(s.camera)) throw std::runtime_error("Invalid camera: "+s.name);
    if (!o.write) {
      s.reference.resize(21*6300);
      std::ifstream input(o.reference/(s.name+".bin"),std::ios::binary);
      input.read(reinterpret_cast<char*>(s.reference.data()),s.reference.size()*sizeof(float));
      if (!input || input.peek()!=std::char_traits<char>::eof()) throw std::runtime_error("Invalid reference tensor: "+s.name);
    }
    loaded.push_back(std::move(s));
  }
  if (loaded.empty()) throw std::runtime_error("No cached image samples");
  return loaded;
}

double milliseconds(Clock::time_point a,Clock::time_point b) {
  return std::chrono::duration<double,std::milli>(b-a).count();
}

// 几何和 PnP 等价性在计时区间之外核对，数值容差不放宽语义一致性。
struct Difference {bool equal=true;double tensor=0,corner=0,position=0,rotation=0;};
Difference difference(const float* raw,const std::vector<float>& reference,
                      const std::vector<ArmorCandidate>& current,const LetterboxTransform& transform,
                      const NnParams& params,const PnpSolver& solver,const CameraCalibration& camera) {
  Difference d;
  for (std::size_t i=0;i<reference.size();++i) {
    const double delta=std::abs(static_cast<double>(raw[i])-reference[i]);
    d.tensor=std::max(d.tensor,delta);
    // 原始张量包含被拒绝的候选；记录差异，验收以最终检测和 PnP 测量等价为准。
    if(!std::isfinite(raw[i]))d.equal=false;
  }
  const auto expected=decodeNnOutput(reference.data(),reference.size(),transform,params);
  if(expected.size()!=current.size()){d.equal=false;return d;}
  for(std::size_t i=0;i<expected.size();++i){
    const auto&a=current[i];const auto&b=expected[i];
    if(a.id!=b.id || a.number_class!=b.number_class || a.is_red!=b.is_red || a.is_large_armor!=b.is_large_armor)d.equal=false;
    for(int j=0;j<4;++j)d.corner=std::max(d.corner,cv::norm(a.corners[j]-b.corners[j]));
    const auto x=solver.solve(a,camera),y=solver.solve(b,camera);
    if(x.valid!=y.valid){d.equal=false;continue;}
    if(!x.valid)continue;
    d.position=std::max(d.position,cv::norm(x.pose.tvec-y.pose.tvec));
    cv::Matx33d rx,ry;cv::Rodrigues(x.pose.rvec,rx);cv::Rodrigues(y.pose.rvec,ry);
    const auto delta=rx*ry.t();
    const double angle=std::acos(std::max(-1.0,std::min(1.0,(cv::trace(delta)-1)/2)))*180/CV_PI;
    d.rotation=std::max(d.rotation,angle);
  }
  d.equal=d.equal && d.corner<=.01 && d.position<=.001 && d.rotation<=.1;
  return d;
}
}

// 同步调用与生产模型相同的 ORT CPU Session，分别记录预处理、推理和后处理。
int main(int argc,char**argv) {
  try {
    const auto o=options(argc,argv);cv::setNumThreads(o.opencv);
    const auto config=parameters(o);const auto& params=config.first;const PnpSolver solver(config.second);
    auto loaded=samples(o);
    Ort::Env env(ORT_LOGGING_LEVEL_WARNING,"nn_benchmark");Ort::SessionOptions settings;
    settings.SetIntraOpNumThreads(o.threads);settings.SetInterOpNumThreads(1);
    settings.SetExecutionMode(ExecutionMode::ORT_SEQUENTIAL);
    settings.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
    Ort::Session session(env,params.model_path.c_str(),settings);
    auto memory=Ort::MemoryInfo::CreateCpu(OrtArenaAllocator,OrtMemTypeDefault);
    const std::array<int64_t,4> shape{{1,3,480,640}};const char* in[]={"images"};const char* out[]={"output0"};
    if(o.write)fs::create_directories(o.reference);
    std::cout<<"iteration,image,pre_ms,infer_ms,post_ms,total_ms,equivalent,max_tensor_abs,max_corner_px,max_position_m,max_rotation_deg\n"<<std::setprecision(12);
    const int extra=o.write?static_cast<int>(loaded.size()):0;
    for(int index=-o.warmup-extra;index<o.iterations;++index){
      auto& sample=loaded[static_cast<std::size_t>(index+o.warmup+extra)%loaded.size()];
      const auto t0=Clock::now();auto input=prepareNnInput(sample.image,params);
      auto tensor=Ort::Value::CreateTensor<float>(memory,input.blob.ptr<float>(),input.blob.total(),shape.data(),shape.size());
      const auto t1=Clock::now();
      auto output=session.Run(Ort::RunOptions{nullptr},in,&tensor,1,out,1);const auto t2=Clock::now();
      const auto info=output[0].GetTensorTypeAndShapeInfo();
      if(info.GetElementType()!=ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT ||
         info.GetShape()!=std::vector<int64_t>({1,21,6300}))throw std::runtime_error("Unexpected model output");
      const auto raw=output[0].GetTensorData<float>();
      auto armors=decodeNnOutput(raw,21*6300,input.transform,params);const auto t3=Clock::now();
      if(o.write && index<-o.warmup){
        sample.reference.assign(raw,raw+21*6300);
        std::ofstream file(o.reference/(sample.name+".bin"),std::ios::binary);
        file.write(reinterpret_cast<const char*>(raw),21*6300*sizeof(float));
        if(!file)throw std::runtime_error("Cannot write reference");
      }
      if(index<0)continue;
      const auto d=difference(raw,sample.reference,armors,input.transform,params,solver,sample.camera);
      std::cout<<index<<','<<sample.name<<','<<milliseconds(t0,t1)<<','<<milliseconds(t1,t2)<<','
        <<milliseconds(t2,t3)<<','<<milliseconds(t0,t3)<<','<<d.equal<<','<<d.tensor<<','<<d.corner<<','<<d.position<<','<<d.rotation<<'\n';
    }
    std::cerr<<"ORT "<<OrtGetApiBase()->GetVersionString()<<"; intra="<<o.threads<<"; OpenCV="<<o.opencv<<"; cached images="<<loaded.size()<<"\n";
  }catch(const std::exception&e){std::cerr<<e.what()<<'\n';return 1;}
}
