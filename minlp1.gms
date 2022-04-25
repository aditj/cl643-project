Sets
    i drones /D1, D2/
    j deliveries /L1,L2,L3/
    k timeslots /T1,T2/;
Scalar delta 'time in a timeslot' / 4 /;
Scalar vlow 'lower bound on the speed achieved by the drone' / 10 /;
Scalar vhigh 'upper bound on the speed achieved by the drone' / 40 /;

Parameters
    d(j) delivery in ms
    / L1 20
      L2 10
      L3 30 /
    tau(j) preferred time slot
    / L1 1
      L2 2
      L3 1 /
    ti(k) time slot index
    /
        T1 1
        T2 2
    /
    w(j) delivery weights
    / L1 2
      L2 5
      L3 4 /
    ;
Table wx(j,k) wind in direction of path in location j at timeslot k
        T1 T2
    L1  2  5 
    L2  6  2
    L3  5  1;
Table wy(j,k) wind perpendicular to direction of path in location j at timeslot k
        T1 T2
    L1  2  1 
    L2  3  4
    L3  1  2;
Variable
        z  total power consumed;

Positive Variables
    
    v(j,k) i speed used  for delivery j at timeslot k
    alpha(j,k) Drone i direction for delivery j at timeslot k
    t(j,k) time taken to deliver j at time t;
Binary Variable
    x(i,j) Drone i makes delivery to Location j
    y(j,k) Location j is delivered in Timeslot k ;
Equations
    cost define objective function
    single_drone(j) ensuring only a single drone visits one
    visit_once(j) ensuring each location is visited once
    timetaken(k) the time taken or delivering in a single slot should be less than the slot size
    opposewind(j,k) oppose wind speed
    preferredtimeslot(j,k) constraint for checking preffered time slot
   ;
v.up(j,k) = vhigh;
v.lo(j,k) = vlow;
alpha.up(j,k) = 6.28;
alpha.lo(j,k) = 0.0;
cost.. z =e= sum((j,k),((y(j,k)*v(j,k))**2)*w(j));
opposewind(j,k).. v(j,k)*sin(alpha(j,k))=e=wy(j,k)*(y(j,k));
single_drone(j).. sum(i,x(i,j)) =e= 1;
** add x(i,j) consistency for time taken 
visit_once(j).. sum(k,y(j,k)) =e= 1;
timetaken(k).. sum(j,y(j,k)*d(j)/(v(j,k)*cos(alpha(j,k))+wx(j,k))) =l=  delta;
preferredtimeslot(j,k).. (ti(k)-tau(j))*y(j,k) =e= 0;
** correct preferred timeslot     preferredtimeslot(j) ensuring that deliveries get made in 

Model ddp / all /;
solve ddp using minlp minimizing z;

    