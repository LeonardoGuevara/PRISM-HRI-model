# PROBABILISTIC MODELS OF HRI IN AGRICULTURE
This repository contains a probabilistic model of Human-Robot Interactions (HRIs) for three different agricultural scenarios. The agricultural scenarios modeled  here include: UV-C treatment, automatic picking, logistics during harvesting operations. These scenarios are modelled as Markov Decision Processes (MDPs) and translated to PRISM language in order to use them in PRISM model checking tool.

The repository consist of two files, the model itself is contained into the file `HRI_PRISM_model`, while the file `Properties_to_evaluate` contains a list of properties that can be used to evaluate the probability of getting human injuries during HRIs. The aim of performing a human injury assesment is to quantify the negative consequences of potential failures in the robot safety system.

Note that in order to use both files, it is neccesary to install `PRISM model checker` (follow the guidelines given [here](https://www.prismmodelchecker.org/download.php)).
