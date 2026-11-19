#include "detectoradapter.h"
#include <QCoreApplication>
#include <QTimer>
#include <iostream>
#include <fstream>
#include <sstream>
int main(int argc, char** argv) {
 QCoreApplication app(argc, argv);
 qputenv("PEOPLE_PROBE_TRACE", "1");
 if (argc > 1) {
  InputCloud::Ptr input(new InputCloud), output;
  std::ifstream csv(argv[1]); std::string line; std::getline(csv,line);
  while(std::getline(csv,line)) { for(char& c:line) if(c==',') c=' '; std::istringstream row(line); InputPoint p; if(row>>p.x>>p.y>>p.z) {p.intensity=0; input->push_back(p);} }
  if(input->empty()) return 2;
  PeopleDetector::Config config;
  if (argc > 2) { config.enable_edge_rejection=false; config.min_range=0.5f; config.max_range=8.0f; }
  PeopleDetector detector(config); PeopleDetector::Diagnostics d;
  auto people=detector.detect(input,output,&d);
  std::cout << "replay raw=" << d.raw_points << " ground=" << d.ground_removed_points << " clusters=" << d.clusters << " height_reject=" << d.rejected_height << " width_reject=" << d.rejected_width << " depth_reject=" << d.rejected_depth << " accepted=" << d.accepted << std::endl;
  for(const auto& p:people) std::cout << "person center=" << p.center.transpose() << " dimensions=" << p.width << ',' << p.depth << ',' << p.height << " points=" << p.points << std::endl;
  return 0;
 }
 DetectorAdapter adapter;
 unsigned long frames=0;
 QObject::connect(&adapter, &DetectorAdapter::statusChanged, [](const QString& s){std::cout << s.toStdString() << std::endl;});
 QObject::connect(&adapter, &DetectorAdapter::errorOccurred, [](const QString& s){std::cerr << s.toStdString() << std::endl;});
 QObject::connect(&adapter, &DetectorAdapter::frameReady, [&](const DetectionFrame& f){
  ++frames;
  if (frames % 10 != 1) return;
  const auto& d=f.diagnostics;
  std::cout << "frame=" << f.frameNumber << " raw=" << d.raw_points << " roi=" << d.roi_points
   << " voxel=" << d.voxel_points << " ground=" << d.ground_removed_points << " remaining=" << d.non_ground_points
   << " clusters=" << d.clusters << " reject_points=" << d.rejected_points << " reject_height=" << d.rejected_height
   << " reject_width=" << d.rejected_width << " reject_depth=" << d.rejected_depth << " accepted=" << d.accepted << std::endl;
  std::ofstream cloud("probe_processed.csv"); cloud << "x,y,z\n";
  for(const auto& p:f.points) cloud << p.x << ',' << p.y << ',' << p.z << '\n';
 });
 QTimer::singleShot(30000, &app, &QCoreApplication::quit);
 adapter.startDetection();
 app.exec();
 adapter.stopDetection(); adapter.wait();
 std::cout << "Received frames=" << frames << std::endl;
 return frames ? 0 : 2;
}
