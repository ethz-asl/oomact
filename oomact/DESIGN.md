# Design considerations

## The model
The *model* represent all components that don't change over time (set of sensors, links , joints, free body trajectories).
It owns only settings and priors for its variables.
It is composed of modules added at construction time or specified through configuration.

## Calibrator
The *calibrator* owns all the variables and measurements used in calibration (depending on the phase it is in) together with the settings for the calibration itself.
It supports *plugins*, which are loaded on demand by type (unique per plugin type).


Problems:

ModuleStorage and inheritance: how to support covariant storage. 
Ideas: 
	* Having a virtual link getter? (too slow)
->	* Having multiple storage entries (the storage links could become the keys? Resolution is a bit slow, but could be cached in the links; maybe add invalidation events).


? PointCloud policy: defined by a calibrator plugin for each sensor.
	* The storage could have factory methods incorporate more arguments. ISSUE : where to get the current data for it; could be Calibrator as entry point
	* The storage could come with extra payload (extra template parameter) ISSUE: getter names! (could be compensated with functions operating on the storage such as getPluginFrom()
	* storage is replaced with something else , e.g. observation manager with getStorage() and getPlugin() / getCalibrator() ..
	* Virtual inheritance ISSUE : slow

	
## Calibrator Plugins


## Calibrator Phases
The calibrator's activity is divided into phases. Modules / Plugins can register to take part in these phases.
Currently they do so by implementing interfaces.

