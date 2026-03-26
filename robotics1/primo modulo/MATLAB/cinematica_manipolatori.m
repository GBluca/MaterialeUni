clear all
% close all
clc


manipolatore = 'antropomorfo';
% planare
% sferico
% antropomorfo
% polso_sferico
% antropomorfo_polso_sferico
% otherwise 

switch manipolatore
    
        case {'planare3B'}
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %        Manipolatore Planare a 3 bracci        %
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        %syms theta1 theta2 theta3
        theta1 = 0; theta2 = 0;  theta3 = 0; 
        d1 = 0; d2 = 0; d3 = 0; a1= .5; a2 = .3;  a3 = .3; 
        L_P(1) = Link([theta1 d1 a1 0 0]);
        L_P(2) = Link([theta2 d2 a2 0 0]);
        L_P(3) = Link([theta3 d3 a3 0 0]);
        L_P
        MP3 = SerialLink(L_P,'name','Manipolatore Planare 3 bracci')
        links = MP3.links
        MP3.plot([pi/4 pi/4 pi/4])
    
    case {'planare2B'}
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %               Manipolatore Planare            %
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        %syms theta1 theta2 theta3
        theta1 = 0; theta2 = 0; 
        d1 = 0; d2 = 0; a1= 1; a2 = 1; 
        L_P(1) = Link([theta1 d1 a1 0 0]);
        L_P(2) = Link([theta2 d2 a2 0 0]);
        L_P
        MP2 = SerialLink(L_P,'name','Manipolatore Planare 2 bracci')
        links = MP2.links

        % calcolo cinematica diretta
        
        theta1 = pi/6; theta2 = pi/6; 
        q = [theta1 theta2]; % forma simbolica
        T = MP2.fkine(q)
        %MP.plot(q, 'workspace', [-4 4 -4 4 -4 4],'tilesize',.5) % in presenza di giunti prismatici occorre definire il workspace del manipolatore
        %MP2.teach(q, 'workspace', [-4 4 -4 4 -4 4],'tilesize',.5)
        
        %q_calc1 = MP.ikinem(T,[0.1 0.1 0.1]); % q0, options: ‘qlimits’ permette di specificare i limiti sulle variabili di giunto
        q_calc2 = MP2.ikunc(T,[0.1 0.1]);
        %[q; q_calc1; q_calc2]
        %MP2.plot(q_calc2, 'workspace', [-3 3 -3 3 -3 3]) % in presenza di giunti prismatici occorre definire il workspace del manipolatore
        
        J = MP2.jacob0(q)        % può essere calcolato anche in forma simbolica
        theta = 0:.01:2*pi;
        rho = 1;
        q1dot = rho*cos(theta); q2dot = rho*sin(theta);
        %qnorm = []
        size_v= length(theta)
        for i=1:size_v;
            v(:,i) = J*[q1dot(i) q2dot(i)]';
        end
        
        pdot_norm = v(1,:).^2 + v(2,:).^2 + v(3,:).^2;
        min_idx = find(pdot_norm == min(pdot_norm));
        max_idx = find(pdot_norm == max(pdot_norm));
        
        w_norm = v(4,:).^2 + v(5,:).^2 + v(6,:).^2;
        
        figure(2), plot(q1dot, q2dot)
        hold on
        daspect('auto')
        arrow3([0 0],[q1dot(max_idx ) q2dot(max_idx)])
        arrow3([0 0],[q1dot(min_idx ) q2dot(min_idx)],'r')
        
        grid on, axis equal
        
        figure(3), plot(v(1,:), v(2,:)), grid on, axis equal
        hold on
        daspect('auto')
        arrow3([0 0],[v(1,max_idx ), v(2,max_idx)])
        arrow3([0 0],[v(1,min_idx ), v(2,min_idx)],'r')
        grid on, axis equal
        
        
        
    case {'planare'}
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %               Manipolatore Planare            %
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        %syms theta1 theta2 theta3
        theta1 = 0; theta2 = 0; theta3 = 0; 
        d1 = 0; d2 = 0; d3 = 0; a1= 1; a2 = 1; a3 = 1;
        L_P(1) = Link([theta1 d1 a1 0 0]);
        L_P(2) = Link([theta2 d2 a2 0 0]);
        L_P(3) = Link([theta3 d3 a3 0 0]);
        L_P
        MP = SerialLink(L_P,'name','Manipolatore Planare 3 bracci')
        links = MP.links

        % calcolo cinematica diretta
        
        theta1 = pi/6; theta2 = pi/6; theta3 = pi/6;
        q = [theta1 theta2 theta3]; % forma simbolica
        T = MP.fkine(q)
        MP.plot(q, 'workspace', [-4 4 -4 4 -4 4],'tilesize',.5) % in presenza di giunti prismatici occorre definire il workspace del manipolatore
        MP.plot(q, 'workspace', [-3 3 -3 3 -1 1],'tilesize',.5) % in presenza di giunti prismatici occorre definire il workspace del manipolatore
        %MP.teach(q, 'workspace', [-4 4 -4 4 -4 4],'tilesize',.5)
        
        %q_calc1 = MP.ikinem(T,[0.1 0.1 0.1]); % q0, options: ‘qlimits’ permette di specificare i limiti sulle variabili di giunto
        q_calc2 = MP.ikunc(T,[0.1 0.1 0.1]);
        %[q; q_calc1; q_calc2]
        %MP.plot(q_calc2, 'workspace', [-3 3 -3 3 -3 3]) % in presenza di giunti prismatici occorre definire il workspace del manipolatore
        
        J = MP.jacob0(q)        % può essere calcolato anche in forma simbolica
        
    
    case {'sferico'}
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %               Manipolatore Sferico            %
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        %syms theta1 theta2 d3
        theta1 = 0; theta2 = 0; d3 = 0; d2 = 1;
        L_MS(1) = Link([theta1 0 0 -pi/2]);
        L_MS(2) = Link([theta2 d2 0 pi/2]);
        L_MS(3) = Link([0 d3 0 0 1]);
        L_MS
        
        MS = SerialLink(L_MS,'name','Manipolatore Sferico')
        %MS.n        % numero giunti
        %links = MS.links
        
        % calcolo cinematica diretta
        %theta1 = pi/6; theta2 = pi/6; d3 = 2;
        q = [theta1 theta2 d3];
        T = MS.fkine(q)
        MS.plot(q, 'workspace', [-3 3 -3 3 -3 3],'tilesize',.5) % in presenza di giunti prismatici occorre definire il workspace del manipolatore
        
        %q_calc1 = MS.ikinem(T,[0.1 0.1 0.1]); % q0, options: ‘qlimits’ permette di specificare i limiti sulle varia bili di giunto
        q_calc2 = MS.ikunc(T,[0.1 0.1 0.1]);
        %[q; q_calc1; q_calc2]
        %MS.plot(q_calc1, 'workspace', [-3 3 -3 3 -3 3]) % in presenza di giunti prismatici occorre definire il workspace del manipolatore
        
        J = MS.jacob0(q)        % può essere calcolato anche in forma simbolica
        
    case {'antropomorfo'}
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %          Manipolatore Antropomorfo            %
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        %a2 = 1; a3 = 1;
        
        %syms theta1 theta2 theta3  a2 a3
        theta1=0; theta2=0; theta3=0;  
        a2=1; a3=1;
        %L(1) = Link([theta1 2 0 pi/2]);
        % d1 = 2 --> offset della terna 0, per sollevare il manipolatore rispetto al piano;  cambia però la matrice di trasformazione
        L_MA(1) = Link([theta1 0 0 pi/2]);
        L_MA(2) = Link([theta2 0 a2 0]);
        L_MA(3) = Link([theta3 0 a3 0]);
        
        MA = SerialLink(L_MA,'name','Manipolatore Antropomorfo')
        
        % calcolo cinematica diretta
        %theta1=pi/3; theta2=pi/6; theta3=pi/6;
        theta1 = 70*pi/180;
        theta2 = 20*pi/180;
        theta3 = 20*pi/180;
        q = [theta1 theta2 theta3];
        
        %T1 = MA.fkine(q);
        MA.plot(q,'workspace', [-4 4 -4 4 -2 3],'tilesize',.5)   % 'workspace', [-3 3 -3 3 0 3] CON OFFSET
        
        [T1,A] = MA.fkine(q);
        hold on                                                 % traccia tutte le terne
        for i=1:2
            h(i)= plot(A(i), 'frame', num2str(i),'rgb');        % in alternativa: trplot
        end
         
%%%     Eliminare i commenti, aggiunti per provare a modificare il comando teach        
        %q_calc1 = MA.ikinem(T1,[0.1 0.1 0.1]); % q0, options: ‘qlimits’ permette di specificare i limiti sulle varia bili di giunto
        q_calc2 = MA.ikunc(T1,[0.1 0.1 0.1]);
        % [q; q_calc1; q_calc2]
        MA.plot(q_calc2, 'workspace', [-3 3 -3 3 -3 3],'tilesize',.5) % in presenza di giunti prismatici occorre definire il workspace del manipolatore
        
        J = MA.jacob0(q)        % può essere calcolato anche in forma simbolica
                        
        
    case {'polso_sferico'}
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %               Polso Sferico                   %
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        %syms theta4 theta5 theta6  d6
        theta4=0; theta5=0; theta6=0;  d6=0.5;
        L_PS(1) = Link([theta4 0 0 -pi/2]);
        L_PS(2) = Link([theta5 0 0 pi/2]);
        L_PS(3) = Link([theta6 d6 0 0]);
        
        PS = SerialLink(L_PS,'name','Polso Sferico')
        
        % calcolo cinematica diretta
        theta4=-pi/3; theta5=pi/6; theta6=-pi/3;
        q = [theta4 theta5 theta6];
        
        T2 = PS.fkine(q);
        PS.plot(q,'workspace', [-3 3 -3 3 -2 3],'tilesize',.5)   % 'workspace', [-3 3 -3 3 0 3] CON OFFSET
        
        %q_calc1 = PS.ikinem(T2,[0.1 0.1 0.1]); % q0, options: ‘qlimits’ permette di specificare i limiti sulle varia bili di giunto
        q_calc2 = PS.ikunc(T2,[0.1 0.1 0.1]);
        %[q; q_calc1; q_calc2]
        %PS.plot(q_calc1, 'workspace', [-3 3 -3 3 -3 3],'tilesize',.5) % in presenza di giunti prismatici occorre definire il workspace del manipolatore
        
        J = PS.jacob0(q)        % può essere calcolato anche in forma simbolica
        
    case {'antropomorfo_polso_sferico'}
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %  Manipolatore Antropomorfo con Polso Sferico  %
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        %syms theta1 theta2 theta3 theta4 theta5 theta6 a2 d4 d6
        theta1=0; theta2=0; theta3=0; theta4=0; theta5=0; theta6=0;
        a2=2; d4=2; d6=0.5;
        
        L_MAPS(1) = Link([theta1 0 0 pi/2]);
        L_MAPS(2) = Link([theta2 0 a2 0]);
        L_MAPS(3) = Link([theta3 0 0 pi/2]);
        L_MAPS(4) = Link([theta4 d4 0 -pi/2]);
        L_MAPS(5) = Link([theta5 0 0 pi/2]);
        L_MAPS(6) = Link([theta6 d6 0 0]);
        
        MAPS = SerialLink(L_MAPS,'name','Manipolatore Antropomorfo Polso Sferico')
        
        % calcolo cinematica diretta
        theta1=-pi/3; theta2=pi/6; theta3=0; theta4=pi/4; theta5=pi/6; theta6=pi/4;
        q = [theta1 theta2 theta3 theta4 theta5 theta6];
        
        T3 = MAPS.fkine(q);
        MAPS.plot(q,'workspace', [-5 5 -5 5 -3 2],'tilesize',.6)

        %q_calc1 = MAPS.ikinem(T3,[0.1 0.1 0.1 0.1 0.1 0.1]); % q0, options: ‘qlimits’ permette di specificare i limiti sulle varia bili di giunto
        q_calc2 = MAPS.ikunc(T3,[0.1 0.1 0.1 0.1 0.1 0.1]);
        %[q; q_calc1; q_calc2]
        MAPS.plot(q_calc2, 'workspace', [-5 5 -5 5 -3 2],'tilesize',.6) % in presenza di giunti prismatici occorre definire il workspace del manipolatore

        J = MAPS.jacob0(q)        % può essere calcolato anche in forma simbolica
        
    otherwise
        
end

