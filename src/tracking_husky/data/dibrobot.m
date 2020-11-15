function h=dibrobot(location, attributes, lado, color)
%GLN 2005
%Pinta el robot en la posición que se le indique
%function dibrobot(location, attributes, lado, color)
%location: [x y chi]
%attributes: Ej attibutes: 'r-'   'b.'
%lado: (optativo) tamaño del robot
%color: (optativo) Ej color: [0.1 1 0.5]

if (nargin < 3),
	lado=0.4;
end

lado_coseno = lado*cos(location(3)-pi/2) ;
lado_seno   = lado*sin(location(3)-pi/2) ;

%Modelo robot: 
%              |x
%              |
%        b-----e-----c
%        |    /|\    |
%        |   / | \   |
%  y ----------0  \  |
%        | /       \ |
%        |/         \|
%        a-----------d
       
       
modelo= [-1  1  1 %a
          1  1  1 %b
          1 -1  1 %c
         -1 -1  1 %d
         -1  1  1 %a
          1  0  1 %e
         -1 -1  1 %d
          ];
     
tranformacion= [lado_coseno  -lado_seno    location(1);
                lado_seno     lado_coseno  location(2);
                0             0            1         ];

robot= [tranformacion*modelo']';

if (nargin < 4),
	h=plot(robot(:,1), robot(:,2), attributes);
else
    h=plot(robot(:,1), robot(:,2), attributes, 'color',color);
end


