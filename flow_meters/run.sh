#!/usr/bin/with-contenv bashio

bashio::log.info "Starting Flow Meters add-on..."

bashio::log.info "DEBUG: /dev/i2c* listing:"
ls -la /dev/i2c* 2>&1 || true
bashio::log.info "DEBUG: end listing"

exec python3 /flow_meters.py
