# Floor calibration markers

Download [the printable PDF](markers_DICT_4X4_50_15cm.pdf). It contains ArUco DICT_4X4_50 IDs **40 and 41**.

Print at **100% / actual size**, with fit-to-page disabled. Measure the **black square** on each page:
it must be **15 cm × 15 cm**, excluding the white border. Keep the white border intact.

Mount the pages flat on stiff material if needed and place both flat on the floor, separated and fully
visible to the main camera. Keep them still and avoid glare. Choose marker 40 as the forward direction
from the camera; avoid placing it immediately below the lens. Check that the dashboard sees IDs 40 and 41.

With the robot controller stopped, capture the floor calibration from the repository:

```sh
.venv/bin/python tools/floor_from_markers.py --ids 40 41 --forward-id 40 --size-cm 15
```

Validate the reported distances with a tape measure. Recalibrate if the camera moves. This calibrates
the floor plane only: tags on moving speaker heads also need calibrated joint and marker-to-chassis
geometry before navigation can use them as chassis position and heading. See [HEAD_TRACKER.md](../../docs/HEAD_TRACKER.md).

Regenerate the printouts with:

```sh
.venv/bin/python tools/generate_markers.py --ids 40-41 --size-cm 15 --out markers/floor-36cm
```
