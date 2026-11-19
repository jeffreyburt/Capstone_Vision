# Third-party assets

The detector runs entirely in the connected browser. These files are bundled so internet access is not needed during operation.

| Component | Version / source |
| --- | --- |
| MediaPipe Tasks Vision JavaScript and WebAssembly | `@mediapipe/tasks-vision` **0.10.21**, from https://cdn.jsdelivr.net/npm/@mediapipe/tasks-vision@0.10.21/ |
| BlazeFace short-range model | Float16 revision **1**, from https://storage.googleapis.com/mediapipe-models/face_detector/blaze_face_short_range/float16/1/blaze_face_short_range.tflite |

MediaPipe is a Google project distributed under the Apache License 2.0. Preserve the existing copyright/license notices in the bundled files and the accompanying [Apache 2.0 license](third_party/mediapipe-LICENSE).

Sources:

- https://github.com/google-ai-edge/mediapipe
- https://www.npmjs.com/package/@mediapipe/tasks-vision/v/0.10.21
- https://ai.google.dev/edge/mediapipe/solutions/vision/face_detector
- https://storage.googleapis.com/mediapipe-assets/MediaPipe%20BlazeFace%20Model%20Card%20%28Short%20Range%29.pdf

The application uses Picamera2 and pyserial from the Pi's system packages. Those dependencies are not vendored into this repository and retain their own licenses.
