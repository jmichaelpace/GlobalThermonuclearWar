#!/bin/bash

# Test fire control radar implementation
# Run simulation for 30 seconds and check for intercept activity

echo "=== Fire Control Radar Test ==="
echo "Starting simulation..."

# Run for 30 seconds then kill
timeout 30s cargo run --release 2>&1 | tee /tmp/fire_control_test.log

# Analyze results
echo ""
echo "=== Test Results ==="
echo ""

# Count track establishments
tracks=$(grep -c "3+ measurements" /tmp/fire_control_test.log 2>/dev/null || echo "0")
echo "Tracks with 3+ measurements: $tracks"

# Count intercepts attempted
intercepts=$(grep -c "calculate_intercept_solution" /tmp/fire_control_test.log 2>/dev/null || echo "0")
echo "Intercept calculations attempted: $intercepts"

# Check for any error messages
errors=$(grep -c "Error\|Failed\|panic" /tmp/fire_control_test.log 2>/dev/null || echo "0")
echo "Errors encountered: $errors"

# Show final status
echo ""
echo "Simulation ran for up to 30 seconds"
echo "Log saved to: /tmp/fire_control_test.log"
