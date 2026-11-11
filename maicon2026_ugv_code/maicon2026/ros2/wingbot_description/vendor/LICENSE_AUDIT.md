# Vendored source and license evidence

The 11 binary STL files in `meshes/ugv_rover/`, `vendor/ugv_rover.urdf`, and
`vendor/package.xml` were copied from the user-provided
`ugv_ws-ros2-humble-develop.zip`, subdirectory
`src/ugv_main/ugv_description`. The package metadata names the package
`ugv_description` and declares `<license>BSD</license>`.

No standalone LICENSE or COPYRIGHT file was present in that package in the
provided archive. This record preserves the actual declaration; it does not
invent a BSD variant, copyright holder, or permission text. The copied metadata
and byte hashes in `source_manifest.json` are the provenance evidence. Confirm
the applicable license text with Waveshare before external redistribution.

The user-supplied WINGBOT measurements, photo-fit data and competition photos
are separate local reference materials. The competition document prohibits
external sharing. Keep this package and its reference images local/private.

`vendor/ugv_rover.gazebo.sdf` and `vendor/ugv_gazebo.package.xml` come from
`src/ugv_main/ugv_gazebo`, whose copied package metadata also declares BSD.
The generator reads the original LiDAR sensor's local +0.04 m pose from that
SDF. It does not load the legacy Classic plugins or mesh collisions.
