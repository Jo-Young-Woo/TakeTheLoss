# Rendering and measured UAV calibration

The canonical `arena_spec.yaml` and `layout_default.yaml` remain byte-for-byte copies of the supplied inputs. The camera's optical mounting yaw and field of view are explicit profile assumptions: `uav_camera.mount_yaw_offset = π/2`, `optical_hfov = 130°`. This aligns image right/up with world +X/+Y and includes the raised outer walls/corner boxes, rather than only the floor plane. The 6 m validation camera is separate and disabled in normal simulation.

## Ogre 1 compatibility

Actual CPU and GPU Ogre 1 captures initially hid textured blocks behind the floor. Untextured versions rendered correctly; replacing meshes with primitives, using distinct mesh files, rebasing cluster links, disabling lighting, and changing material scripts did not fix the textured scene. The missing blocks followed camera-distance sorting, consistent with opaque textures being put in a transparent pass without depth writes. All source PNGs already contained RGB pixels without alpha.

`maicon_opaque_material_system` repairs only package-owned textured materials on Gazebo's `SceneUpdate` rendering-thread event. It disables texture alpha/transparency, then enables depth checking and depth writing. Each actual material is repaired once; weak cached references permit resource recreation. Geometry, physical collisions, shared meshes, UVs and photographs remain unchanged. `topdown_opaque_plugin_gpu.png` is the successful actual GPU A/B capture showing every cube and sensor box.

The plugin is included only for `render_engine=ogre`; the default Ogre 2 world is unchanged. Generator option `--opaque-material-compat false` disables it for diagnosis. Build with the ROS package before launching Ogre 1. The default library target is Fortress; `-DMAICON_GAZEBO_SIMULATOR=harmonic` selects the corresponding library family, which has not been runtime-tested here.

The separately running GUI client now has `MaiconOpaqueMaterialGui`, which applies the identical cached repair on its own rendering-thread `PreRender` event. The mandatory `GzSceneManager` imports the server scene; `MinimalScene` alone only creates an empty viewport. Interactive view and camera tracking are invisible helpers, with the scene, controls and statistics remaining the visible interface.

An actual **CPU Ogre 2 sensor server + GPU Ogre 1 GUI** run was visually verified with the full floor print, all 20 layout cubes, the hazard boxes and six UGVs visible (`validation/gpu_gui_verified.png`). GPU Ogre 2 with a GL 4.3 override remains experimental: an opaque repair restored geometry but did not restore correct atlas/lighting appearance. Valid LiDAR alone is not acceptance of that renderer. The validated split uses GPU for the GUI and CPU for sensor rendering; it is not an all-GPU claim.

For an explicit GUI render-loop measurement, set `MAICON_GUI_METRICS_PATH` to the desired JSON path only in the GUI process. After a 5 s warm-up it counts `PreRender` callbacks in sixty fixed one-second wall-clock buckets and writes the report once, atomically. There is no per-frame disk I/O and no output when that variable is absent. Actual hybrid SHM64 measurements gave standard mean/minimum one-second render-loop rates of **137.23 / 96 Hz**, and lite **143.40 / 121 Hz**. Mean/minimum real-time factors were standard **0.999059 / 0.994690**, lite **0.999998 / 0.999620**, with startup times **2.596 / 2.635 s** respectively. These callback rates are **render-loop measurements, not display-present FPS**. They exceed 30 Hz at the measured render hook; display presentation was not independently measured.

## Actual side-image close-ups

`scripts/probe_arena_closeups.py` added two temporary, collision-free cameras to the running CPU Ogre 2 world, received three real ROS image frames per camera, and removed both models afterward. The arena and robot poses were unchanged. The evaluator derived face centres from the byte-identical canonical YAML; this information was not supplied to an autonomous agent. It captured the east face of `cube_0_0` (`target:outpost`) and the east face of `cube_14_0` (`m3:military_truck`) from 0.65 m away at face-centre height.

Both captures are upright and unmirrored. After a crop determined from camera intrinsics and the physical 0.4 m face size, the source-image normalized correlations were **0.98595** (target) and **0.98150** (M3); the strongest reflected/rotated alternatives were **0.59467 / 0.55484**. Gazebo lighting darkens the images without reversing them. `validation/closeups/closeup_report.json` records image hashes, camera poses and all orientation comparisons. The `*_source_vs_capture.png` images show the supplied source at left and the actual rendered face crop at right. This verifies two sampled physical faces, not recognition performance or every face instance. The regression test also deliberately reflects and rotates the captured frames and confirms those faults fail the acceptance check.

## Calibration evidence

`validation/uav_render_gpu_6ugv.png` is the actual 1280×720, 130° capture with six UGVs. `config/uav_grid_calibration.json` was fitted from mutually matched real image features of the known floor print. The print's measured 6.00×4.80 m size independently maps reference-image pixels to metric coordinates. These are printed-floor feature measurements distributed across the arena, not a claim that every visible mark is a grid-line intersection.

The fit/hold-out split is fixed before fitting. No point is created by projecting fitted corners or a homography; no box layout or simulator object pose is supplied. RANSAC accepted 23 of 24 fit points and rejected one ambiguous repeated print motif. The 23 held-out measurements have RMS error **2.375 mm**, maximum **6.439 mm**, within the 1 cm validation tolerance. The fit residual is training evidence and is reported separately. See `validation/uav_grid_overlay.png`, `uav_grid_measurements.json`, `uav_grid_checkpoints.json`, and `uav_calibration_detection.json`.

The calibration frame has its origin at the print's bottom-left: X∈[0,6], Y∈[0,4.8]. Convert to centred Gazebo coordinates by subtracting (3,2.4). The optical centre is (3,2.4,2.5) in that frame. Box-top projection uses the separate z=0.4 m plane to account for parallax. Inputs are undistorted pinhole images; drawing a grid does not estimate lens distortion. A changed camera pose, field of view, crop or resolution requires new measured calibration and hold-out checks.

The separate actual **6 m validation-camera** capture `topdown_render_6ugv.png` also passes the 1 cm floor-alignment check. Its 28 measured correspondences contain 26 RANSAC inliers; 27 separate held-out matches have RMS **1.919 mm**, maximum **5.873 mm**. Checking those same held-out pixels directly against the specified 6 m camera pose/FOV, **without the fitted homography**, gives RMS **5.083 mm**, maximum **8.424 mm**. That additional check cannot absorb a misplaced floor by refitting a homography. Evidence is stored separately in `topdown_grid_calibration.json`, `topdown_grid_overlay.png` and `topdown_calibration_detection.json`; the 2.5 m UAV calibration was not overwritten.

Measurement review is explicit: the first 6 m attempt at descriptor ratio 0.5 had one false repeated-print correspondence among its held-out matches and was rejected (`topdown_initial_fit_rejected.json`). A stricter descriptor uniqueness criterion of 0.4 was then applied before a fresh split and fit. No checkpoint was removed using its final residual. This is reviewed calibration evidence, not a blind detector benchmark.

Reproduce from the repository root:

```bash
python3 scripts/calibrate_arena_uav_fixture.py \
  --image ros2/maicon_arena_gz/validation/uav_render_gpu_6ugv.png \
  --overwrite

python3 scripts/calibrate_arena_uav_fixture.py \
  --image ros2/maicon_arena_gz/validation/topdown_render_6ugv.png \
  --camera-height 6 --evidence-prefix topdown --max-match-ratio 0.4 \
  --nominal-camera-profile ros2/maicon_arena_gz/config/profiles/standard.yaml \
  --output ros2/maicon_arena_gz/validation/topdown_grid_calibration.json --overwrite
```

## Image-only inspection candidates

The updated `FixedUavLayoutDetector` preserves legacy grey-top behavior and processes black tops separately. Blank black side faces project toward the camera and can merge with their top. Anchoring 0.4 m cells at the contour edge farthest from the camera, then requiring each cell to be filled by actual image evidence, recovers the top footprints without treating the side projection as another object.

The captured frame yields **20/20** distinct layout-cube candidates; each is 0.4×0.4 m, and offline comparison puts every centre within 1.2 cm of the layout. Runtime inference does not read that evaluator layout. The regression test forbids file reads while detection runs, verifies a blank frame yields no candidates, and checks synthetic image motion and rejection of a UGV-sized patch. Legacy captured fixtures still return their original 13/17 rectangles.

The actual CPU Ogre 2 HTTP JPEG (`validation/api_uav_live.jpg`) also yields **20/20** candidates, including through HTTP → asynchronous delivery → Agent coverage. JPEG's tiny near-black channel differences can have high HSV saturation; the initial saturation gate incorrectly broke these black regions and recovered only two cubes. Black segmentation now uses absolute brightness (maximum RGB channel ≤18), with no saturation gate near zero, while grey segmentation remains separate. Tests cover both captured PNG/JPEG views, noisy black pixels, and unchanged legacy 13/17 outputs. See `validation/api_uav_cube_candidates.png` and `api_uav_detection.json`. The integration test verifies that these hints create neither M3 class counts nor hidden SLAM obstacles.

`validation/uav_cube_candidates.png` shows all candidates. They carry geometry/confidence only: M3 class and side labels remain **unknown** until ground-camera inspection. Twenty overhead candidates do not mean twenty M3 objects or complete M3 scoring. This evidence covers one fixed-lighting, single-layer, axis-aligned scene; occlusion, higher stacks, shadows and real-camera appearance need additional validation and ground-only fallback.
