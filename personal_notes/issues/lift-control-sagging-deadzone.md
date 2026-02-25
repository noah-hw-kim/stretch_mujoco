# Issue: Lift control sagging and deadzone

## Problem
Lift drifts downward under small upward commands.

## Experiments
- experiments/2026-01-08-lift-sagging-and-cmd-meas-discrepancy.md
- experiments/2026-02-10-lift-sagging-investigation.md

## Findings
- meas + delta unstable
- cmd integrator stable but lagged
- move_by has minimum effective delta ≈ 0.008–0.010 m
- deadzone is upward-only

## Current Design
- move_by
- dt = 0.05
- upward-only minimum delta guard
- scale_lift ≈ 0.3

## Status
Active (tuning action mapping)