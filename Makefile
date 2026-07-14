# universal-form gates. Every claim in the README is meant to be a gate you can run.
# Scripts import the `kinematics` package from the repo root, so we run with PYTHONPATH=.

PY ?= python3
RUN = PYTHONPATH=. $(PY)

.PHONY: check ik-check reach-check

check: ik-check reach-check   ## every hardware-free gate

ik-check:      ## closed-form IK exactly inverts FK (FK.IK = identity to 1e-9)
	$(RUN) scripts/ik_check.py

reach-check:   ## base+lift deliver reach the arm alone cannot (base delivers, arm settles)
	$(RUN) scripts/reach_check.py
