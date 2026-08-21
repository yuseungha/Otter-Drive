# Competition runbook

## Before connecting drive power

1. Verify the physical emergency-stop operator and power cutoff.
2. Confirm the BRIO and Arduino stable device paths in `.env`.
3. Run `./scripts/run_competition.sh --check`.
4. Run `./scripts/run_competition.sh --dry-run` and inspect lane topics.
5. Lift the driven wheels and verify neutral output, steering sign, limits,
   watchdog, deadman, communication loss, and `Ctrl+C` shutdown.
6. Only then set `KMU_HARDWARE_CONFIRMED=true` in `.env`.

## Competition start

```bash
cd /home/sandi/KMU_AutoDriving
./scripts/run_competition.sh --live
```

If any preflight check fails, do not bypass it. Return to DRY-RUN and inspect
the latest directory under `logs/`.

## Stop

Press `Ctrl+C`, confirm the vehicle is neutral, then remove drive power. The
runner stops the named container and records the ROS output.
