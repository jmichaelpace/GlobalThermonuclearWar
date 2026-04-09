#!/bin/bash

echo "=== Testing Salvo Fire with Middle East Scenario ==="
echo "Shahab-3 MRBMs have lower apogees (~100-150km) that are within THAAD envelope"
echo ""

# Run simulation and capture salvo fire output
cargo run --release 2>&1 | grep -E "Engaging with|Launch|Followup|Miss" | head -100

echo ""
echo "Note: Start the app and select 'Middle East Crisis' scenario to see salvo fire"
echo "Shahab-3 missiles should trigger THAAD salvo fire (2 interceptors per target)"
