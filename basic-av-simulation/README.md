
I wrote the code for trajectory.jl to mathematically determine a trajectory where cars, represented by rectangles in the 2D, avoid collisions with each other and obey lane boundaries as part of my Autonomous Vehicles project class.

Instructions to run:

Install julia with instructions specific to your operating system. On Mac, I ran 

```
brew install juliaup
```

Clone this repository to your computer. Navigate to the root of this directory, and run 

```
julia --project
```

Then, press `]` to enter `pkg` mode. From this mode, run instantiate, as the following:
```julia
(simple-simulation) pkg> instantiate
```

Press `delete` to return to the normal REPL. From here, we can now import our simple-simulation project.

```julia
julia> using Revise, simple-simulation
```

Notice that I also started "using" a package called Revise. Your REPL may complain that it cannot find this package yet. 
If so (and for any such package that you might need), you can simply add it to the project from `pkg` mode, 

```julia
(simple-simulation) pkg> add Revise
```

Although it is only really necessary for development so this step is not required. 


The visualization.jl file contains a visualization of the trajectory I generated for the blue car to not collide with the other 3 red cars.

To use this, you can run the following lines in julia:


```julia
julia> include("test/test_utils.jl")
julia> (; trajectories, halfspaces) = setup_manual_planning_problem()
julia> simple-simulation.plot_trajectories(trajectories, halfspaces)
```


