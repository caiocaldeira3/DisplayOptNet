# OpticalNet

> OBS: This repository is to be used as a framework for the implementation of re-structuring algortihms in distributed networks.

### Introduction

This project aims to provide a framework of simulation for the operation of an algorithm that restrucures the topology of network
dynamically, during the simulation. In order to be as close as possible of a current real hardware implementation it provides a 
grouping of nodes and edges of our network within optical switches. Currently this repository provides this grouping for networks from the
BST family, however an extension is soon to be implemented to allow graphs of arbitrary maximum degree.

We consider an Optical Switch to be a simple data relay with the same number of input and output ports, connected to the nodes of our network.
For disclosure, $S(m)$, in this context means that this switch has $m$ input ports and $m$ output ports, each connected to a different node. Each
switch does not have any knowledge of another switch within the simulation. These switches are grouped in what we call Clusters, and each cluster, 
with the input-output connections, represent one sub-graph of our network.

The simulation is divided in rounds and each round is divided in timeslots. This is done to keep synchronization among the nodes within the simulation,
note that this is one of the abstractions this framework makes in favor of simplicity. These timeslots deal, respectively, which nodes want to send a 
message or alter their links, with the decision process of which node can be active this round, the actual routing or altering process, and, finally,
a logging timeslot, meant to save the round operations in the simulation file.

The Controller Node, the central part of the simulation and [similar in concept to a disco-ball](https://www.univie.ac.at/ct/stefan.old/osn21.pdf), 
then is responsible to decide which nodes can be active in a given round and to inform each node which operations they must do to pass through their 
message. The major part of the computational process is centralized in the controller node, as the other involved parts, nodes or switches, are considered
to be mainly data transmitters.

### Plotting Tools
As said before, after each round this framework will log the round results in the simulation file. We provide some plotting tools with which the interested
party can analyze their results over some of what we consider to be the most important aspects of efficiency a re-structuring network algorithm is
bound to.

To use them the user must only update the number of nodes, switches, simulations and the dataset name on `plot_script.py`.

### Requirements
This project uses Java and Sinalgo to run the simulations. In order to use the scripts available it is needed to have `numpy` installed as well as 
`python>=3.9.7`. The plot scripts use `numpy`, `pandas`, `statsmodels` and [SciencePlots](https://github.com/garrettj403/SciencePlots) for stylization.
