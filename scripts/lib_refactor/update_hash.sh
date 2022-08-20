#!/usr/bin/env bash

find ./resources -name "*.conf" | xargs -n1 ./utils/gen_hash.sh
