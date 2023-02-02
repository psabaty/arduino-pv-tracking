#!/bin/sh
BASEDIR=$(dirname "$0")
PACKAGES_DIR=$BASEDIR/packages

rm -rf $BASEDIR/var/arduino_build
mkdir $BASEDIR/var/arduino_build
rm -rf $BASEDIR/var/arduino_cache
mkdir $BASEDIR/var/arduino_cache

echo "Precomp..."

arduino-builder -dump-prefs -logger=machine -verbose -warnings=default \
    -hardware /usr/share/arduino/hardware -hardware $PACKAGES_DIR \
    -tools /usr/share/arduino/hardware/tools/avr -tools $PACKAGES_DIR \
    -libraries $PACKAGES_DIR/arduino/hardware/avr/1.8.6/libraries \
    -fqbn=arduino:avr:uno -ide-version=10819 \
    -prefs=build.warn_data_percentage=75 \
    -prefs=runtime.tools.avr-gcc.path=$PACKAGES_DIR/arduino/tools/avr-gcc/7.3.0-atmel3.6.1-arduino7 \
    -prefs=runtime.tools.avr-gcc-7.3.0-atmel3.6.1-arduino7.path=$PACKAGES_DIR/arduino/tools/avr-gcc/7.3.0-atmel3.6.1-arduino7 \
    -prefs=runtime.tools.arduinoOTA.path=$PACKAGES_DIR/arduino/tools/arduinoOTA/1.3.0 \
    -prefs=runtime.tools.arduinoOTA-1.3.0.path=$PACKAGES_DIR/arduino/tools/arduinoOTA/1.3.0 \
    -prefs=runtime.tools.avrdude.path=$PACKAGES_DIR/arduino/tools/avrdude/6.3.0-arduino17 \
    -prefs=runtime.tools.avrdude-6.3.0-arduino17.path=$PACKAGES_DIR/arduino/tools/avrdude/6.3.0-arduino17 \
    -build-path $BASEDIR/var/arduino_build \
    -build-cache $BASEDIR/var/arduino_cache \
    $BASEDIR/phil.ino

#exit 
echo "Compiling..."

arduino-builder -compile -logger=machine -verbose -warnings=default \
    -hardware /usr/share/arduino/hardware -hardware $PACKAGES_DIR \
    -tools /usr/share/arduino/hardware/tools/avr -tools $PACKAGES_DIR \
    -libraries $PACKAGES_DIR/arduino/hardware/avr/1.8.6/libraries \
    -fqbn=arduino:avr:uno -ide-version=10819 \
    -prefs=build.warn_data_percentage=75 \
    -prefs=runtime.tools.avr-gcc.path=$PACKAGES_DIR/arduino/tools/avr-gcc/7.3.0-atmel3.6.1-arduino7 \
    -prefs=runtime.tools.avr-gcc-7.3.0-atmel3.6.1-arduino7.path=$PACKAGES_DIR/arduino/tools/avr-gcc/7.3.0-atmel3.6.1-arduino7 \
    -prefs=runtime.tools.arduinoOTA.path=$PACKAGES_DIR/arduino/tools/arduinoOTA/1.3.0 \
    -prefs=runtime.tools.arduinoOTA-1.3.0.path=$PACKAGES_DIR/arduino/tools/arduinoOTA/1.3.0 \
    -prefs=runtime.tools.avrdude.path=$PACKAGES_DIR/arduino/tools/avrdude/6.3.0-arduino17 \
    -prefs=runtime.tools.avrdude-6.3.0-arduino17.path=$PACKAGES_DIR/arduino/tools/avrdude/6.3.0-arduino17 \
    -build-path $BASEDIR/var/arduino_build \
    -build-cache $BASEDIR/var/arduino_cache \
    $BASEDIR/phil.ino


#    -libraries /home/psabaty/Arduino/libraries \
#    -build-path /tmp/arduino_build_592824 \
#    -build-cache /tmp/arduino_cache_537481 \
