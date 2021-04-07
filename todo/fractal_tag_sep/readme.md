# Fractal Tag Samples

- sample_landing

```shell
./build/yujie_sample_landing ./resources/landing.mov ./resources/landingPad.png
./build/yujie_sample_detection ./resources/image_with_markers.jpeg
./fractal_tracker video.avi -cam cameraParams.yml -c FRACTAL_3L_6
```
- read from [official doc](https://docs.google.com/document/d/1SdsOTjGdu5o8gy2Ot2FDqYDS9ALgyhOBJcJHOZBR7B4/edit)

```shell
# new example for fractal
./build/fractal_tracker my_video.mkv -cam cameraParams.yml -c FRACTAL_4L_6 -s 0.2
./fractal_tracker ../my_video.mkv -cam ../cameraParams.yml -c FRACTAL_4L_6 -s 0.2
```