close all
clear all 

delimiterIn = ' ';
headerlinesIn = 0;

filename = 'q_alfa.dat';
alfa = importdata(filename,delimiterIn,headerlinesIn);


filename = 'vel.dat';
velocidades = importdata(filename,delimiterIn,headerlinesIn);



filename = 'yaw.dat';
angulos = importdata(filename,delimiterIn,headerlinesIn);

filename = 'error_tracking.dat';
et = importdata(filename,delimiterIn,headerlinesIn);




obstacles=[-10 10;-5 -5; 3 -3; 10 0;-5.12 -0.3; 5.53 -8.228;-0.176 -4.12;-3.306 4.17;5.327 1.83;7.7183 -4.972;0.6664 -7.48;5.69 6];
nob=size(obstacles);
Robj=[1.5];
frec=40;
endtime=120;
N=endtime*frec;
figure(1);
h=figure(1);

%subplot(2,3,1);
nrob=1;
hold on

grid on
    
        i=1;
        plot(alfa(1:N,i), alfa(1:N,i+nrob),'k-', 'LineWidth', 1.5);
        om=200;
        %m1=4*sin(2*2*pi/om*(1:N)).*cos(2*pi/om*(1:N));
        %m2=4*sin(2*2*pi/om*(1:N)).*sin(2*pi/om*(1:N));
        %plot(m1,m2,'-m','Linewidth',1);
        plot(alfa(1,i),alfa(1,i+nrob),'k-x', 'LineWidth', 2.5); 
        plot(alfa(N,i),alfa(N,i+nrob),'m-o', 'LineWidth', 1.5);
        
    
    
    dibrobot([alfa(N,1),alfa(N,nrob+1),angulos(N,1)+pi/2], 'k-',0.5);
    for i=1:nob(1)
        circle(obstacles(i,1),obstacles(i,2),Robj(1));
        circle(obstacles(i,1),obstacles(i,2),.1);
        dibrobot([obstacles(i,1),obstacles(i,2),0], 'g-',0.5);
    end
    
    
	
    dibrobot([alfa(N,1),alfa(N,nrob+1),angulos(N,1)+pi/2], 'k-',0.5);
    dibrobot([alfa(1,1),alfa(1,nrob+1),angulos(1,1)+pi/2], 'k-',0.5);
    %dibrobot([alfa(1,5),alfa(1,10),angulos(1,5)+pi/2], 'r-',0.1);
    
    for j=1:30:N
        
    dibrobot([alfa(j,1),alfa(j,nrob+1),angulos(j,1)+pi/2], 'k-',0.5);
        
    end
    dibrobot([alfa(N,1),alfa(N,nrob+1),angulos(N,1)+pi/2], 'k-',0.5);
    %dibrobot([alfa(N,5),alfa(N,10),angulos(N,5)+pi/2], 'r-',0.1);
    
   
xlabel('Distancia (metros)') 
ylabel('Distancia (metros)')
  legend({'Trayectoria realizada','Referencia'},'Location','northwest')  
title('Trayectoria de agente')
hold off

figure('DefaultAxesFontSize',15)
figure(2);
title('Velocidad lineal')
xlabel('Tiempo (segundos)') 
ylabel('Velocidad (mts/seg)')
%subplot(2,3,2);
hold on
grid
    
    plot(0:1/frec:(N-1)/frec, velocidades(1:1:N,2),'k-');
  %  t = linspace(0,(N-1)/frec);
 %       axes('position',[.3 .6 .4 .25])
%box on % put box around new pair of axes
%plot(20:1/frec:35,velocidades(800:1:1400,2)) % plot on new axes
%axis tight

hold off

figure('DefaultAxesFontSize',15)
figure(3);
%subplot(2,3,5);
xlabel('Tiempo (segundos)') 
ylabel('Velocidad (rad/seg)')
title('Velocidad angular')
hold on
grid
        
        plot(0:1/frec:(N-1)/frec, velocidades(1:1:N,3),'k-');
        %t = linspace(0,(N-1)/frec);
        %axes('position',[.3 .6 .4 .25])
%box on % put box around new pair of axes
%plot(20:1/frec:35,velocidades(800:1:1400,3)) % plot on new axes
%axis tight
        

hold off

figure('DefaultAxesFontSize',15)

figure(4);
%subplot(2,3,6);
hold on
grid
    plot(0:1/frec:(N-1)/frec, et(1:1:N,2),'r-');
    plot(0:1/frec:(N-1)/frec, et(1:1:N,3),'b-');
    %plot(1:N, consenso(1:N,10),'r-');
    xlabel('Tiempo (segundos)') 
    %ylabel('Error de tracking')
legend('Error en X','Error en Y')
title('Error de tracking')
hold off




