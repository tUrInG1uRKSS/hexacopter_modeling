function plot_all(t, X, angles, model_type)

figure;
subplot(3,1,1); plot(t, X(:,1:3)); title("Position (m)");
legend("x","y","z");

subplot(3,1,2); plot(t, X(:,4:6)); title("Velocity (m/s)");
legend("vx","vy","vz");

subplot(3,1,3); plot(t, X(:,7:9)); title("Angular rates (rad/s)");
legend("p","q","r");

figure;
plot(t, angles(:,1:3)); title("Euler Angles (rad)");
legend("\phi","theta","\psi");
grid on;

end
