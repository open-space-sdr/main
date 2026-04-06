#!/bin/bash
cd ~/demos/

# Default to the KasmVNC virtual X11 display
MPV_VO_FLAG="--vo=x11"

# Check ONLY the HDMI ports for a physical monitor, ignoring MIPI/DSI nodes
for status_file in /sys/class/drm/*HDMI*/status; do
  if [ -f "$status_file" ] && grep -q "^connected$" "$status_file" 2>/dev/null; then
    # An HDMI monitor is plugged in! Force DRM directly to the console.
    MPV_VO_FLAG="--vo=drm"
    
    # Unset the X11 DISPLAY variable so mpv doesn't try to draw 
    # a window in KasmVNC when the HDMI monitor is present.
    unset DISPLAY
    break
  fi
done


./ntsc_demod --bypass_iir true --disc atan2 --no_deemph --read_samps 65536 --flush_frames 1 \
  --args "numBuffers=2,bufferLength=65536" \
  --diag_hz 2 --hsync_min 25 --hsync_max 160 --sat 3.0 --hue -3.0 \
| mpv --profile=low-latency --no-cache \
  --demuxer-thread=no --vd-lavc-threads=1 \
  --demuxer=rawvideo --demuxer-rawvideo-w=640 --demuxer-rawvideo-h=480 \
  --demuxer-rawvideo-mp-format=yuyv422 --demuxer-rawvideo-fps=60 $MPV_VO_FLAG -


#./ntsc_demod --bypass_iir true --disc atan2 --no_deemph --read_samps 65536 --flush_frames 1 \
#  --args "numBuffers=2,bufferLength=65536" \
#  --diag_hz 2 --hsync_min 25 --hsync_max 160 \
#  --sat 3.0 --hue -3.0 \
#| ffmpeg -hide_banner -y -f rawvideo -pix_fmt yuyv422 -s 640x480 -r 60 -i - \
#  -c:v mjpeg -q:v 5 -f mpjpeg "tcp://0.0.0.0:1234?listen=1"

# To record to file,
#./ntsc_demod --bypass_iir true --disc atan2 --no_deemph --read_samps 65536 --flush_frames 1 \
#  --args "numBuffers=2,bufferLength=65536" \
#  --diag_hz 2 --hsync_min 25 --hsync_max 160 \
#  --sat 3.0 --hue -3.0 > test_output.yuv

# And play back,
# ffplay -f rawvideo -pixel_format yuyv422 -video_size 640x480 -framerate 60 -loop 0 test_output_long.yuv


sleep 3
