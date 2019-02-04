classdef Quadcopter
    %% Quadcopter class for simulation of the paper: Cooperative object transportation in 3D
    %% with multiple quadrotors using no peer communication
    %%
    properties
        %%
        d, alpha, c, r; % structural property of quadcopter. d is relative to the main object
        max_thrust; % vector containing the max thrust a motor can exert
        min_thrust; % '' min thrust ''
        W; Q2O; R; % matrix (13)
    end
    
    methods
        function this = Quadcopter(d, alpha, c, r)
            %% constructor
            this.d = d;
            this.alpha = alpha;
            this.c = c;
            this.r = r;
            this.max_thrust = [5, 5, 5, 5];
            this.min_thrust = [0, 0, 0, 0];
            
            C = cos(this.alpha);
            S = sin(this.alpha);
            
            
            this.R = [1 0 0 0;
                      0 C S 0;
                      0 -S C 0;
                      0 0 0 1];
            this.W = [                        1                          1                          1                          1;
                           (-this.r*C+this.r*S)        (this.r*C-this.r*S)        (this.r*C+this.r*S)       (-this.r*C-this.r*S);
                     (this.d+this.r*C+this.r*S) (this.d-this.r*C-this.r*S) (this.d+this.r*C-this.r*S) (this.d-this.r*C+this.r*S);
                                        this.c                     this.c                    -this.c                    -this.c]; % matrix (13)
            this.Q2O = [1   0 0 0; 
                       -d*S 1 0 0;
                        d*C 0 1 0;
                        0   0 0 1;]*[1  1  1  1;
                                    -r  r  r -r;
                                     r -r  r -r;
                                     c  c -c -c];
        end
        
        function f = computeThrusts(this, wrench)
            %% computes sigle motor thrusts from the modified wrench
            wrench = this.R*wrench;
            p = this.computeP(wrench);
            tau_y = this.computeModifiedTau(wrench, p);
            f = this.findThrusts(wrench, tau_y, p);
        end

        function tau_y = computeModifiedTau(this, wrench, p)
            %% computes the modified Tau_y. (25) of the article
            if wrench(3) < 0
                tau_y = wrench(3);
            else
                tau_y = 2*wrench(3) + this.d*wrench(1) - p ;
            end
        end
        
        function f = findThrusts(this, wrench, tau_y, p)
            %% compute minimization problem (24).
            C = cos(this.alpha);
            S = sin(this.alpha);           
            Aeq(1:2,:) = this.W(1:2,:);
            Aeq(3,:) = this.W(4,:);
            beq = [wrench(1), wrench(2), wrench(4)]';
            A_ineq = [this.W(3,:);-this.W(3,:)];
            b_ineq = [(this.d*wrench(1) + p);...
                      (-this.d*wrench(1) + p)];
                  
            H = 2*this.W(3,:)'*this.W(3,:);
            F = -2*this.W(3,:)*tau_y;
            
%             options = optimset('Algorithm','interior-point-convex','Display','off');
%             f = quadprog(H,F,A_ineq, b_ineq, Aeq, beq, this.min_thrust, this.max_thrust, [], options);
            [f,~,~,~,~,~] = qpOASES(H,F',[Aeq;this.W(3,:)],this.min_thrust',this.max_thrust',[beq; (this.d*wrench(1) - p)],[beq;(this.d*wrench(1) + p)]); 
        end
        function p = computeP(this, wrench)
            C = cos(this.alpha);
            S = sin(this.alpha);
            H = eye(4);
            F = [this.r*(C+S), -this.r*(C+S), this.r*(C-S), -this.r*(C-S)];
                 
            Aeq(1:2,:) = this.W(1:2,:);
            Aeq(3,:) = this.W(4,:);
            beq = [wrench(1), wrench(2), wrench(4)]';
            
%             options = optimset('Algorithm','interior-point-convex' , 'Display','on');
%             f_min = quadprog(H,F,[], [], Aeq, beq, this.min_thrust, this.max_thrust, [], options);
%             f_max = quadprog(H,-F,[], [], Aeq, beq, this.min_thrust, this.max_thrust, [], options);
            [f_min,~,~,~,~,~] = qpOASES(H,F',Aeq,this.min_thrust',this.max_thrust',beq,beq); 
            [f_max,~,~,~,~,~] = qpOASES(H,F',Aeq,this.min_thrust',this.max_thrust',beq,beq); 
            
            p_min = F*f_min ;
            p_max = F*f_max ;
            p = min([abs(p_min), abs(p_max)]);
        end
        function Wquad = computeLocalWrench(this, wrench)
           f = this.computeThrusts(wrench);
           Wquad = this.Q2O*f;
        end
    end
end

