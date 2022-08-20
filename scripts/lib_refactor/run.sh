#!/usr/bin/env bash

find ./resources -name "*.conf" | xargs -n1 ./utils/build.sh
