// Cross-session global localization harness.
//
// The map comes from one MulRan session, the query scans from another. For each
// query frame the FPFH + RANSAC stages run once and produce one candidate set;
// every polish configuration is then applied to that same set, so the
// configurations differ only in the polish step.
//
// Usage: see README.md. All arguments are key=value.

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "global_localization.hpp"
#include "map_tile.hpp"
#include "mulran_sequence.hpp"

namespace
{

struct Arguments {
  std::string map_sequence = "/home/cgt24/work/dataset/rosBagFiles/mulran/kaist001";
  std::string query_sequence = "/home/cgt24/work/dataset/rosBagFiles/mulran/kaist002";
  std::string output_path = "out/polish_compare.csv";
  std::string tag; // prefix on progress lines, e.g. "[3/9]"
  long start_frame = 0;
  long frame_count = -1; // -1 = to the end
  long stride = 10;
  int threads = 16;
  int progress_every = 10;
  glexp::MapTileParams tile;
  glexp::LocalizerParams localizer;
  std::vector<glexp::PolishParams> configs;
  // One extra alignment per frame started from the ground truth pose. It
  // measures how far the two sessions' ground truth disagree, which bounds how
  // tight a pass threshold means anything.
  bool gt_probe = true;
};

[[noreturn]] void fail(const std::string& message) {
  std::cerr << "error: " << message << std::endl;
  std::exit(1);
}

std::vector<std::string> split(const std::string& text, char separator) {
  std::vector<std::string> parts;
  std::stringstream stream(text);
  std::string part;
  while (std::getline(stream, part, separator)) {
    parts.push_back(part);
  }
  return parts;
}

// "icp,dist=2.0,iter=20" / "ndt,res=2.0,iter=30" / "none"
// num_threads is left at -1 unless the config overrides it, so the run-wide
// threads= argument can fill it in afterwards whatever the argument order was.
glexp::PolishParams parsePolishConfig(const std::string& spec) {
  const std::vector<std::string> parts = split(spec, ',');
  if (parts.empty()) {
    fail("empty polish config");
  }
  glexp::PolishParams params;
  params.num_threads = -1;
  if (!glexp::parsePolishMethod(parts[0], &params.method)) {
    fail("unknown polish method: " + parts[0]);
  }
  for (std::size_t i = 1; i < parts.size(); ++i) {
    const std::size_t equals = parts[i].find('=');
    if (equals == std::string::npos) {
      fail("polish option needs key=value: " + parts[i]);
    }
    const std::string key = parts[i].substr(0, equals);
    const std::string value = parts[i].substr(equals + 1);
    if (key == "dist") {
      params.max_correspondence_distance = std::stod(value);
    } else if (key == "iter") {
      params.max_iterations = std::stoi(value);
    } else if (key == "eps") {
      params.transformation_epsilon = std::stod(value);
    } else if (key == "res") {
      params.ndt_resolution = std::stod(value);
    } else if (key == "step") {
      params.ndt_step_size = std::stod(value);
    } else if (key == "outlier") {
      params.ndt_outlier_ratio = std::stod(value);
    } else if (key == "k") {
      params.correspondence_randomness = std::stoi(value);
    } else if (key == "reps") {
      params.rotation_epsilon = std::stod(value);
    } else if (key == "lambda") {
      params.init_lambda_factor = std::stod(value);
    } else if (key == "points") {
      params.polish_points = std::stoi(value);
    } else if (key == "threads") {
      params.num_threads = std::stoi(value);
    } else if (key == "fullmap") {
      params.full_map_target = value != "0";
    } else if (key == "label") {
      params.label = value;
    } else {
      fail("unknown polish option: " + key);
    }
  }
  return params;
}

/**
 * GICP with exactly what the plugin already configures for reg_loc_ at
 * e9d662db, aligned against the whole map session the way loadGlobalMap binds
 * it. Nothing here is a free parameter: every value is read off the plugin.
 */
glexp::PolishParams pluginDefaultConfig() {
  glexp::PolishParams params;
  params.method = glexp::PolishMethod::Gicp;
  params.num_threads = -1; // nano_gicp defaults to omp_get_max_threads()
  params.max_correspondence_distance = 1.0; // relocalization.gicp in params.yaml
  params.max_iterations = 32; // gicp_max_iter_
  params.correspondence_randomness = 16; // gicp_k_correspondences_
  params.transformation_epsilon = 0.01; // gicp_transformation_ep_
  params.rotation_epsilon = 0.01; // gicp_rotation_ep_
  params.init_lambda_factor = 1e-9; // gicp_init_lambda_factor_
  params.full_map_target = true;
  params.label = "gicp_plugin";
  return params;
}

Arguments parseArguments(int argc, char** argv) {
  Arguments arguments;
  for (int i = 1; i < argc; ++i) {
    const std::string argument = argv[i];
    const std::size_t equals = argument.find('=');
    if (equals == std::string::npos) {
      fail("arguments must be key=value: " + argument);
    }
    const std::string key = argument.substr(0, equals);
    const std::string value = argument.substr(equals + 1);

    if (key == "map") {
      arguments.map_sequence = value;
    } else if (key == "query") {
      arguments.query_sequence = value;
    } else if (key == "out") {
      arguments.output_path = value;
    } else if (key == "tag") {
      arguments.tag = value;
    } else if (key == "start") {
      arguments.start_frame = std::stol(value);
    } else if (key == "count") {
      arguments.frame_count = std::stol(value);
    } else if (key == "stride") {
      arguments.stride = std::stol(value);
    } else if (key == "threads") {
      arguments.threads = std::stoi(value);
    } else if (key == "progress") {
      arguments.progress_every = std::stoi(value);
    } else if (key == "tile_radius") {
      arguments.tile.radius = std::stod(value);
    } else if (key == "tile_voxel") {
      arguments.tile.voxel_size = std::stod(value);
    } else if (key == "tile_grid") {
      arguments.tile.grid_size = std::stod(value);
    } else if (key == "tile_min_frames") {
      arguments.tile.min_frames = std::stoi(value);
    } else if (key == "top_k") {
      arguments.localizer.top_k = std::stoi(value);
    } else if (key == "query_voxel") {
      arguments.localizer.query_voxel_size = std::stod(value);
    } else if (key == "normal_radius") {
      arguments.localizer.normal_estimation_radius = std::stod(value);
    } else if (key == "fpfh_radius") {
      arguments.localizer.fpfh_search_radius = std::stod(value);
    } else if (key == "budget") {
      arguments.localizer.ransac_matching_budget = std::stoi(value);
    } else if (key == "min_inlier") {
      arguments.localizer.min_inlier_fraction = std::stod(value);
    } else if (key == "corr_dist") {
      arguments.localizer.max_correspondence_distance = std::stod(value);
    } else if (key == "config") {
      arguments.configs.push_back(
        value == "gicp_plugin" ? pluginDefaultConfig() : parsePolishConfig(value));
    } else if (key == "gt_probe") {
      arguments.gt_probe = value != "0";
    } else {
      fail("unknown argument: " + key);
    }
  }
  arguments.localizer.num_threads = arguments.threads;
  if (arguments.configs.empty()) {
    arguments.configs.push_back(parsePolishConfig("none"));
  }
  for (glexp::PolishParams& config : arguments.configs) {
    if (config.num_threads < 0) {
      config.num_threads = arguments.threads;
    }
    if (config.label.empty()) {
      config.label = glexp::defaultPolishLabel(config);
    }
  }
  return arguments;
}

double rotationErrorDegrees(const Eigen::Matrix4d& estimate, const Eigen::Matrix4d& truth) {
  const Eigen::Matrix3d relative = truth.block<3, 3>(0, 0).transpose() * estimate.block<3, 3>(0, 0);
  const double trace = std::min(3., std::max(-1., relative.trace()));
  return std::acos(std::min(1., std::max(-1., (trace - 1.) * 0.5))) * 180. / M_PI;
}

double yawDegrees(const Eigen::Matrix4d& pose) {
  return std::atan2(pose(1, 0), pose(0, 0)) * 180. / M_PI;
}

// Running tally per polish configuration, used for the progress lines and the
// closing summary.
struct Tally {
  std::string label;
  int frames = 0;
  int pass = 0; // < 2 m and < 5 deg
  int gross = 0; // >= 10 m
  double polish_ms_total = 0.0;
  // Target-side setup is kept out of polish_ms: on a robot it is paid once when
  // the map is loaded, not per localization request.
  double target_prep_ms_total = 0.0;
  int target_binds = 0;
  std::vector<double> translation_errors;

  double percentile(double fraction) const {
    if (translation_errors.empty()) {
      return std::nan("");
    }
    std::vector<double> sorted = translation_errors;
    std::sort(sorted.begin(), sorted.end());
    const std::size_t index = std::min(sorted.size() - 1,
      static_cast<std::size_t>(fraction * static_cast<double>(sorted.size())));
    return sorted[index];
  }
};

// pcl::IterativeClosestPoint has no OpenMP path, so its num_threads is
// meaningless; nano_gicp and pclomp NDT honour theirs. Reported per config so
// the timing columns can be read without guessing.
int effectivePolishThreads(const glexp::PolishParams& config) {
  switch (config.method) {
    case glexp::PolishMethod::None:
      return 0;
    case glexp::PolishMethod::Icp:
      return 1;
    case glexp::PolishMethod::Gicp:
    case glexp::PolishMethod::Ndt:
      return config.num_threads;
  }
  return config.num_threads;
}

std::string formatSeconds(double seconds) {
  std::ostringstream text;
  text << std::fixed << std::setprecision(0) << seconds << "s";
  return text.str();
}

} // namespace

int main(int argc, char** argv) {
  const Arguments arguments = parseArguments(argc, argv);

  std::cout << std::fixed;
  std::cout << "map    " << arguments.map_sequence << "\n"
            << "query  " << arguments.query_sequence << "\n"
            << "tile   radius=" << arguments.tile.radius << " voxel=" << arguments.tile.voxel_size
            << " grid=" << arguments.tile.grid_size << " min_frames=" << arguments.tile.min_frames
            << "\n"
            << "ransac budget=" << arguments.localizer.ransac_matching_budget
            << " top_k=" << arguments.localizer.top_k
            << " min_inlier=" << arguments.localizer.min_inlier_fraction
            << " threads=" << arguments.threads << "\n";
  for (std::size_t i = 0; i < arguments.configs.size(); ++i) {
    const glexp::PolishParams& config = arguments.configs[i];
    std::cout << "config " << i << "  " << config.label << "  threads="
              << effectivePolishThreads(config)
              << (config.method == glexp::PolishMethod::Icp ? " (PCL ICP is single threaded)" : "")
              << "  target=" << (config.full_map_target ? "full map" : "tile")
              << "  polish_points=" << (config.polish_points > 0 ? config.polish_points : 0)
              << "\n";
  }
  std::cout << std::flush;

  const Eigen::Matrix4d base_to_lidar = mulran::baseToLidarExtrinsic();
  mulran::Sequence map_sequence(
    arguments.map_sequence, base_to_lidar, arguments.tile.voxel_size, 900);
  mulran::Sequence query_sequence(
    arguments.query_sequence, base_to_lidar, arguments.tile.voxel_size, 4);

  // Both sessions are already in UTM-52N; shifting both by the same origin only
  // keeps the coordinates small enough for float32.
  const Eigen::Vector3d origin = map_sequence.firstGroundTruthPosition();
  map_sequence.setOrigin(origin);
  query_sequence.setOrigin(origin);
  std::cout << "map frames " << map_sequence.size() << " (" << map_sequence.poseCount()
            << " posed), query frames " << query_sequence.size() << " ("
            << query_sequence.poseCount() << " posed)" << std::endl;

  std::vector<long> query_frames;
  const long last_frame = arguments.frame_count < 0
    ? static_cast<long>(query_sequence.size())
    : std::min(static_cast<long>(query_sequence.size()), arguments.start_frame + arguments.frame_count * arguments.stride);
  for (long frame = arguments.start_frame; frame < last_frame; frame += arguments.stride) {
    if (query_sequence.hasPose(static_cast<std::size_t>(frame))) {
      query_frames.push_back(frame);
    }
  }
  std::cout << "evaluating " << query_frames.size() << " query frames (stride "
            << arguments.stride << ")" << std::endl;

  std::ofstream output(arguments.output_path);
  if (!output) {
    fail("cannot write " + arguments.output_path);
  }
  output << std::fixed;
  output << "frame,timestamp_ns,tile_x,tile_y,tile_frames,tile_points,query_points,"
            "gt_x,gt_y,gt_yaw_deg,gt_inlier,gt_probe_ok,gt_probe_dt,gt_probe_dr,gt_probe_inlier,"
            "evaluated,passed,distinct,oracle_dt,feature_ms,ransac_ms,"
            "config,omp_threads,polish_input_points,target_prep_ms,polish_ms,"
            "best_inlier,best_error,err_trans,err_rot_deg,oracle_polished_dt\n";

  glexp::Localizer localizer(arguments.localizer);
  std::vector<std::unique_ptr<glexp::PolishEngine>> engines;
  for (const glexp::PolishParams& config : arguments.configs) {
    engines.push_back(std::make_unique<glexp::PolishEngine>(config));
  }
  // The probe reuses GICP with a generous correspondence distance; it starts at
  // the ground truth pose, so it only has to take up the session-to-session
  // offset, not find the pose.
  glexp::PolishParams probe_params;
  probe_params.method = glexp::PolishMethod::Gicp;
  probe_params.num_threads = arguments.threads;
  probe_params.max_correspondence_distance = 3.0;
  probe_params.max_iterations = 30;
  glexp::PolishEngine probe_engine(probe_params);

  std::vector<Tally> tallies(arguments.configs.size());
  for (std::size_t i = 0; i < arguments.configs.size(); ++i) {
    tallies[i].label = arguments.configs[i].label;
  }

  // Whole-session map, bound once, the way the plugin's loadGlobalMap does it.
  const bool needs_full_map = std::any_of(arguments.configs.begin(), arguments.configs.end(),
    [](const glexp::PolishParams& config) { return config.full_map_target; });
  if (needs_full_map) {
    const auto build_start = std::chrono::steady_clock::now();
    const mulran::Cloud::Ptr full_map =
      glexp::buildFullMap(map_sequence, arguments.tile.voxel_size, 2000);
    const double build_ms =
      std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - build_start)
        .count();
    std::cout << "full map: " << full_map->size() << " pts at " << arguments.tile.voxel_size
              << "m, accumulated in " << std::setprecision(0) << build_ms << "ms" << std::endl;
    for (std::size_t i = 0; i < engines.size(); ++i) {
      if (!arguments.configs[i].full_map_target) {
        continue;
      }
      const auto bind_start = std::chrono::steady_clock::now();
      engines[i]->setTarget(full_map);
      const double bind_ms =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - bind_start)
          .count();
      tallies[i].target_prep_ms_total += bind_ms;
      ++tallies[i].target_binds;
      std::cout << "  bound " << arguments.configs[i].label << " to the full map in "
                << std::setprecision(0) << bind_ms << "ms" << std::endl;
    }
  }

  Eigen::Vector3d current_tile_center = Eigen::Vector3d::Constant(std::nan(""));
  bool tile_ready = false;
  int tile_frames = 0;
  long tile_points = 0;
  int tile_builds = 0;
  int skipped_thin_tiles = 0;
  int frames_done = 0;
  const auto run_start = std::chrono::steady_clock::now();

  std::vector<double> tile_prep_ms(arguments.configs.size(), 0.0);
  for (std::size_t index = 0; index < query_frames.size(); ++index) {
    std::fill(tile_prep_ms.begin(), tile_prep_ms.end(), 0.0);
    const std::size_t frame = static_cast<std::size_t>(query_frames[index]);
    const Eigen::Matrix4d ground_truth = query_sequence.lidarPose(frame);
    const Eigen::Vector3d position = ground_truth.block<3, 1>(0, 3);
    const Eigen::Vector3d center = glexp::tileCenter(position, arguments.tile.grid_size);

    if (!tile_ready || (center.head<2>() - current_tile_center.head<2>()).norm() > 1e-6) {
      const auto tile_start = std::chrono::steady_clock::now();
      int contributing = 0;
      mulran::Cloud::Ptr tile =
        glexp::buildMapTile(map_sequence, center, arguments.tile, -1, 0, &contributing);
      if (contributing < arguments.tile.min_frames) {
        ++skipped_thin_tiles;
        tile_ready = false;
        continue;
      }
      localizer.setMap(tile);
      for (std::size_t i = 0; i < engines.size(); ++i) {
        if (arguments.configs[i].full_map_target) {
          continue; // bound once, before the loop
        }
        const auto bind_start = std::chrono::steady_clock::now();
        engines[i]->setTarget(tile);
        tile_prep_ms[i] = std::chrono::duration<double, std::milli>(
          std::chrono::steady_clock::now() - bind_start)
                            .count();
        tallies[i].target_prep_ms_total += tile_prep_ms[i];
        ++tallies[i].target_binds;
      }
      probe_engine.setTarget(tile);
      current_tile_center = center;
      tile_ready = true;
      tile_frames = contributing;
      tile_points = static_cast<long>(tile->size());
      ++tile_builds;
      const double tile_ms =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - tile_start)
          .count();
      std::cout << "  tile [" << std::setprecision(1) << center.x() << ", " << center.y() << "] "
                << tile_frames << " frames, " << tile_points << " pts, "
                << std::setprecision(0) << tile_ms << "ms" << std::endl;
    }

    const mulran::Cloud::Ptr raw_scan = query_sequence.readScan(frame);
    const glexp::Cloud::ConstPtr query = localizer.downsample(raw_scan);

    glexp::LocalizeStats stats;
    const std::vector<glexp::Candidate> candidates = localizer.localize(query, &stats);

    // How close the candidate stage got, ignoring which candidate the score
    // picked. Separates "RANSAC never proposed the right pose" from "polish or
    // the rescore chose the wrong candidate".
    double oracle_translation = std::nan("");
    for (const glexp::Candidate& candidate : candidates) {
      const double distance = (candidate.transformation.block<3, 1>(0, 3).cast<double>() -
        ground_truth.block<3, 1>(0, 3))
                                .norm();
      if (std::isnan(oracle_translation) || distance < oracle_translation) {
        oracle_translation = distance;
      }
    }

    const Eigen::Matrix4f ground_truth_float = ground_truth.cast<float>();
    const glexp::Candidate at_ground_truth = localizer.score(*query, ground_truth_float);
    double probe_translation = std::nan("");
    double probe_rotation = std::nan("");
    double probe_inlier = std::nan("");
    int probe_ok = 0;
    if (arguments.gt_probe) {
      Eigen::Matrix4f probed = ground_truth_float;
      probe_ok = probe_engine.refine(query, ground_truth_float, &probed) ? 1 : 0;
      probe_translation =
        (probed.block<3, 1>(0, 3) - ground_truth_float.block<3, 1>(0, 3)).cast<double>().norm();
      probe_rotation = rotationErrorDegrees(probed.cast<double>(), ground_truth);
      probe_inlier = localizer.score(*query, probed).inlier_fraction;
    }

    for (std::size_t config_index = 0; config_index < arguments.configs.size(); ++config_index) {
      // Polish may see a thinned scan, but the rescore always sees the whole
      // one, so best_inlier stays comparable across point counts.
      const glexp::Cloud::ConstPtr polish_input = engines[config_index]->thinForPolish(query);
      const auto polish_start = std::chrono::steady_clock::now();
      glexp::Candidate best;
      double oracle_polished = std::nan("");
      for (const glexp::Candidate& candidate : candidates) {
        Eigen::Matrix4f refined = candidate.transformation;
        engines[config_index]->refine(polish_input, candidate.transformation, &refined);
        const glexp::Candidate rescored = localizer.score(*query, refined);
        if (rescored.inlier_fraction > best.inlier_fraction ||
            (rescored.inlier_fraction == best.inlier_fraction && rescored.error < best.error)) {
          best = rescored;
        }
        const double distance =
          (refined.block<3, 1>(0, 3).cast<double>() - ground_truth.block<3, 1>(0, 3)).norm();
        if (std::isnan(oracle_polished) || distance < oracle_polished) {
          oracle_polished = distance;
        }
      }
      const double polish_ms =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - polish_start)
          .count();

      double translation_error = std::nan("");
      double rotation_error = std::nan("");
      Tally& tally = tallies[config_index];
      ++tally.frames;
      tally.polish_ms_total += polish_ms;
      if (!candidates.empty()) {
        const Eigen::Matrix4d estimate = best.transformation.cast<double>();
        translation_error =
          (estimate.block<3, 1>(0, 3) - ground_truth.block<3, 1>(0, 3)).norm();
        rotation_error = rotationErrorDegrees(estimate, ground_truth);
        tally.translation_errors.push_back(translation_error);
        if (translation_error < 2.0 && rotation_error < 5.0) {
          ++tally.pass;
        }
        if (translation_error >= 10.0) {
          ++tally.gross;
        }
      } else {
        // Nothing to polish: counted as a frame, neither a pass nor a gross
        // mismatch, and left out of the error percentiles.
        ++tally.gross;
      }

      output << frame << "," << query_sequence.timestampNs(frame) << ","
             << std::setprecision(2) << center.x() << "," << center.y() << "," << tile_frames
             << "," << tile_points << "," << stats.query_points << "," << position.x() << ","
             << position.y() << "," << yawDegrees(ground_truth) << "," << std::setprecision(4)
             << at_ground_truth.inlier_fraction << "," << probe_ok << "," << probe_translation
             << ","
             << probe_rotation << "," << probe_inlier << "," << stats.evaluated << ","
             << stats.passed << "," << stats.distinct_candidates << "," << std::setprecision(3)
             << oracle_translation << "," << std::setprecision(1) << stats.feature_ms << ","
             << stats.ransac_ms << "," << arguments.configs[config_index].label << ","
             << effectivePolishThreads(arguments.configs[config_index]) << ","
             << polish_input->size() << "," << tile_prep_ms[config_index] << ","
             << polish_ms << "," << std::setprecision(4) << best.inlier_fraction << ","
             << best.error << "," << std::setprecision(3) << translation_error << ","
             << rotation_error << "," << oracle_polished << "\n";
    }
    output.flush();
    ++frames_done;

    if (frames_done % arguments.progress_every == 0 ||
        index + 1 == query_frames.size()) {
      const double elapsed =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - run_start).count();
      const double per_frame = elapsed / static_cast<double>(frames_done);
      const double remaining =
        per_frame * static_cast<double>(query_frames.size() - (index + 1));
      std::cout << arguments.tag << "  " << (index + 1) << "/" << query_frames.size() << "  elapsed "
                << formatSeconds(elapsed) << "  eta " << formatSeconds(remaining) << "  tiles "
                << tile_builds << " (" << skipped_thin_tiles << " thin)" << std::endl;
      for (const Tally& tally : tallies) {
        std::cout << "    " << std::left << std::setw(20) << tally.label << std::right
                  << " pass " << std::setprecision(1)
                  << 100. * static_cast<double>(tally.pass) / std::max(1, tally.frames) << "%"
                  << "  >=10m " << 100. * static_cast<double>(tally.gross) / std::max(1, tally.frames)
                  << "%"
                  << "  p50 " << std::setprecision(2) << tally.percentile(0.5) << "m"
                  << "  p90 " << tally.percentile(0.9) << "m"
                  << "  polish " << std::setprecision(0)
                  << tally.polish_ms_total / std::max(1, tally.frames) << "ms" << std::endl;
      }
    }
  }

  std::cout << "\n=== summary (n=" << frames_done << ") ===" << std::endl;
  std::cout << std::left << std::setw(22) << "config" << std::right << std::setw(8) << "pass%"
            << std::setw(9) << ">=10m%" << std::setw(9) << "p50" << std::setw(9) << "p90"
            << std::setw(11) << "polish_ms" << std::setw(9) << "threads" << std::setw(9)
            << "binds" << std::setw(12) << "prep_ms/bind" << std::endl;
  for (std::size_t i = 0; i < tallies.size(); ++i) {
    const Tally& tally = tallies[i];
    std::cout << std::left << std::setw(22) << tally.label << std::right << std::setprecision(1)
              << std::setw(8) << 100. * static_cast<double>(tally.pass) / std::max(1, tally.frames)
              << std::setw(9) << 100. * static_cast<double>(tally.gross) / std::max(1, tally.frames)
              << std::setprecision(2) << std::setw(9) << tally.percentile(0.5) << std::setw(9)
              << tally.percentile(0.9) << std::setprecision(0) << std::setw(11)
              << tally.polish_ms_total / std::max(1, tally.frames) << std::setw(9)
              << effectivePolishThreads(arguments.configs[i]) << std::setw(9) << tally.target_binds
              << std::setw(12) << tally.target_prep_ms_total / std::max(1, tally.target_binds)
              << std::endl;
  }
  std::cout << "polish_ms excludes target preparation, which a robot pays once at map load."
            << std::endl;
  std::cout << "wrote " << arguments.output_path << std::endl;
  return 0;
}
