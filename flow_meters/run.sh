#!/usr/bin/with-contenv bashio

bashio::log.info "Starting Flow Meters add-on..."

exec python3 /flow_meters.py
