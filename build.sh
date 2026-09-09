#!/usr/bin/env bash
# The project name below is the directory this deploys into on the board, and
# it has to match the three absolute paths in -m or the rpath breaks.
# Bela parses these CLArgs with atoi(), so "--use-digital yes" evaluates to 0 and
# silently turns the pins off. They take numbers, always.
# --stop-button-pin -1 disables the stop button: with all 16 digital channels
# wired, a floating stop pin can kill a session mid-experiment.
# BBB_HOSTNAME="192.168.1.2" ../Bela/scripts/build_project.sh \
BBB_ADDRESS="root@bela.rt" ../Bela/scripts/build_project.sh \
  -p RiseTogether-bela-forwarder \
  -c "--analog-channels 0 --digital-channels 16 --audio-input-gain 0 --line-out-level 0 --hp-level 0 --verbose --mute-speaker 1 --use-analog 0 --use-digital 1 --disable-led --stop-button-pin -1 --period 16 --high-performance-mode" \
  --force \
  -m "CPPFLAGS='-std=c++1z -I/root/Bela/projects/RiseTogether-bela-forwarder/include' LDLIBS=/root/Bela/projects/RiseTogether-bela-forwarder/lib/liblsl.so LDFLAGS='-Wl,-rpath,/root/Bela/projects/RiseTogether-bela-forwarder/lib'" \
  ./
