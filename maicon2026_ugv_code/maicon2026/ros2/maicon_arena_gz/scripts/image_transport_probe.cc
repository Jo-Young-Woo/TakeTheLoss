// Native Fortress image delivery counter. No pixel bytes are printed or saved.
// g++ -std=c++17 image_transport_probe.cc -o image_transport_probe \
//   $(pkg-config --cflags --libs ignition-transport11 ignition-msgs8) -pthread
#include <ignition/transport/Node.hh>
#include <ignition/msgs/image.pb.h>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <functional>
#include <iomanip>
#include <iostream>
#include <map>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

struct Stats
{
  size_t count = 0, duplicate = 0, backwards = 0;
  double first = 0, last = 0, wallFirst = 0, wallLast = 0;
  unsigned width = 0, height = 0;
  size_t bytes = 0;
  std::vector<double> gaps, stamps;
};

int main(int argc, char **argv)
{
  if (argc < 4) { std::cerr << "usage: image_transport_probe seconds output.json topic [topic ...]\n"; return 2; }
  double duration;
  try { duration = std::stod(argv[1]); } catch (...) { return 2; }
  if (!std::isfinite(duration) || duration <= 0 || duration > 3600) return 2;
  using Clock = std::chrono::steady_clock;
  const auto start = Clock::now();
  auto wall = [&]() { return std::chrono::duration<double>(Clock::now()-start).count(); };
  std::map<std::string, Stats> data;
  std::mutex mutex;
  ignition::transport::Node node;
  for (int i = 3; i < argc; ++i) data.emplace(argv[i], Stats{});
  for (int i = 3; i < argc; ++i)
  {
    const std::string topic(argv[i]);
    std::function<void(const ignition::msgs::Image &)> callback = [&, topic](const ignition::msgs::Image &msg)
    {
      const double stamp = msg.header().stamp().sec() + msg.header().stamp().nsec()*1e-9;
      const double received = wall();
      std::lock_guard<std::mutex> guard(mutex);
      auto &s = data.at(topic);
      if (s.count)
      {
        const double gap = stamp-s.last;
        if (gap == 0) ++s.duplicate;
        else if (gap < 0) ++s.backwards;
        else if (s.gaps.size() < 100000) s.gaps.push_back(gap);
      }
      else { s.first = stamp; s.wallFirst = received; }
      ++s.count; s.last = stamp; s.wallLast = received;
      s.width = msg.width(); s.height = msg.height(); s.bytes = msg.data().size();
      if (s.stamps.size() < 64) s.stamps.push_back(stamp);
    };
    if (!node.Subscribe(topic, callback)) { std::cerr << "native subscription failed\n"; return 3; }
  }
  std::this_thread::sleep_for(std::chrono::duration<double>(duration));
  std::lock_guard<std::mutex> guard(mutex);
  const double elapsed = wall();
  std::ofstream out(argv[2]);
  if (!out) return 4;
  out << std::setprecision(12) << "{\"transport\":\"ignition-transport11\",\"wall_seconds\":" << elapsed << ",\"topics\":{";
  bool firstTopic = true;
  for (auto &[topic, s] : data)
  {
    if (!firstTopic) out << ',';
    firstTopic = false;
    std::sort(s.gaps.begin(), s.gaps.end());
    out << std::quoted(topic) << ":{\"count\":" << s.count << ",\"wall_fps\":" << s.count/elapsed
        << ",\"first_stamp\":" << s.first << ",\"last_stamp\":" << s.last
        << ",\"first_received_wall_s\":" << s.wallFirst << ",\"last_received_wall_s\":" << s.wallLast
        << ",\"stamp_span_fps\":" << (s.count > 1 && s.last > s.first ? (s.count-1)/(s.last-s.first) : 0)
        << ",\"duplicate_stamps\":" << s.duplicate << ",\"backwards_stamps\":" << s.backwards
        << ",\"median_positive_stamp_gap_s\":" << (s.gaps.empty() ? 0 : s.gaps[s.gaps.size()/2])
        << ",\"max_stamp_gap_s\":" << (s.gaps.empty() ? 0 : s.gaps.back())
        << ",\"width\":" << s.width << ",\"height\":" << s.height << ",\"bytes\":" << s.bytes
        << ",\"first_64_stamps\":[";
    for (size_t i=0; i<s.stamps.size(); ++i) { if (i) out << ','; out << s.stamps[i]; }
    out << "]}";
  }
  out << "}}\n";
}
