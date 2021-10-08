# Elevator-simulation-system---embedded-system---C
  
Environment: uVision5  
Language: C  
  
Assignment description:  
In class, we have seen how petrinets can be used to expand DFAs to concurrent systems. We  
have also seen how a DFA can be implemented directly into C, using a while loop and a switch  
statement. We have not, however, similarly implemented a petrinet. One issue of doing such  
an implementation is that the transition choices can be nondeterministic, unlike a DFA.  
In a previous posting, I provided PowerPoint slides that give the petrinet for one elevator in a  
three-floor building. (I did not show the 1st and 2nd floors, but they are similar to the 3rd floor:  
1st floor uses🔼instead of 🔽, and 2nd floor uses both.)  
  
In this assignment, you will implement that petrinet. And once you get it working, you will add  
a second elevator. Their behavior will be similar to the behavior of the elevators in the F  
building.   
