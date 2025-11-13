# Physical-Layer-Security-in-Vehicular-Networks

## Updated vehicular/UAV scenario

The repository now contains a MATLAB script (`Code/ScenarioSimulation.m`)
that captures the refined network layout used for the physical-layer
security analysis. The scenario consists of:

* One base station mounted on a short tower.
* Two relays: a car-mounted decode-and-forward relay and a UAV relay
  flying at 150&nbsp;m.
* Three legitimate users (one car user and two UAV users at 150&nbsp;m).
* Two eavesdroppers (one car and one UAV at 150&nbsp;m).

The script models the 3D geometry of the nodes, applies altitude-aware
path-loss exponents for terrestrial, mixed, and aerial links, and
computes the secrecy rate for each legitimate user assuming the best
relay is selected. It also prints a hop-by-hop report for every relay
option, including the base-to-relay and relay-to-user SNRs, the SNR at
each eavesdropper, and the resulting secrecy rate so that **all of the
intermediate results** are visible. Run the script in MATLAB/Octave to
review the summary and detailed tables.
