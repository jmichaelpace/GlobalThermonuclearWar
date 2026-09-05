#!/bin/bash

echo "=== Testing Salvo Fire with Middle East Scenario ==="
echo "Shahab-3 MRBMs have ~430km apogees (config profile: 150 + 0.18*range) -"
echo "above THAAD's 150km ceiling at apogee, so engagements happen on the"
echo "descent leg through the 150-40km band. Israeli layers (Arrow 3, David's"
echo "Sling, Iron Dome) are aimed at the Iran axis via facing_deg."
echo ""

# Run simulation and capture salvo fire output
cargo run --release 2>&1 | grep -E "Engaging with|Launch|Followup|Miss" | head -100

echo ""
echo "Note: Start the app and select 'Middle East Crisis' scenario to see salvo fire"
echo "Shahab-3 missiles should trigger THAAD salvo fire (2 interceptors per target)"
