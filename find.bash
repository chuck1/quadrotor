#!/bin/bash
find .. -name "*.h"   | xargs -i{} grep "$1" {} -Hn
find .. -name "*.c"   | xargs -i{} grep "$1" {} -Hn
find .. -name "*.hpp" | xargs -i{} grep "$1" {} -Hn
find .. -name "*.cpp" | xargs -i{} grep "$1" {} -Hn

