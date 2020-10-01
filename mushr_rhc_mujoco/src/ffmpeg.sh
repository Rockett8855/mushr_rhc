ffmpeg -f rawvideo -pixel_format rgb24 -video_size 1200x900 -framerate 60 -i out.rgb -vf "vflip" video.mp4
