close all

figure(1)
plot3(flighttraj.signals.values(:,1),flighttraj.signals.values(:,2),flighttraj.signals.values(:,3))
hold on
plot3(flighttraj.signals.values(:,4),flighttraj.signals.values(:,5),flighttraj.signals.values(:,6))
axis('equal')
title('Flight Trajectory')
hold off

figure(2)
plot(command.signals.values(:,1))
title('Bank Command - Heading')

figure(3)
plot(command.signals.values(:,2))
title('Throttle Command - Altitude')

figure(4)
plot(command.signals.values(:,3))
title('Elevator Command - Airspeed')

figure(5)
plot(fdynamics.signals.values(:,1))
title('Airspeed')

figure(6)
plot(fdynamics.signals.values(:,4))
title('Bankangle')

figure(7)
plot(fdynamics.signals.values(:,6))
title('Heading')

% save flightdata flighttraj command fdynamics euler