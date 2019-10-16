#!/bin/bash

catkin_make install  --only-pkg-with-deps  crcl_rosmsgs
catkin_make install -DCATKIN_WHITELIST_PACKAGES=""


