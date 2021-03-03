# Marker detection

## Work with camera

- list cameras and its permissions
  
  - `ll /dev/video*`
- cannot access camare
  
  - `chmod 777 /dev/video1`
- sudo apt install guvcview
    1. Check the available resolutions, framerates and formats of your USB camera by running

       ```
       guvcview --device=/dev/video1
       ```

       

    2. Run guvcview and adjust your camera settings (exposure, brightness etc).

---

- whycon-orig
    https://github.com/gestom/whycon-orig/tree/opencv
    ```
    cd src/
    make CXX=clang
    # Error
    ```