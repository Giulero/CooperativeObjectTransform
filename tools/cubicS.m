classdef cubicS
    %CUBICS Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        p_seq;
        v_seq;
        a_seq;
        j_seq;
        v_vec;
        a_vec;
        length;
        points;
        A;
        c;
        m_coeffs;
        delta_t;
        t_vec;
        T_tot;
    end
    
    methods
        function obj = cubicS(vector, T_tot)
            obj.delta_t = 0.02;
            obj.T_tot = T_tot;
            obj.length = length(vector);
            obj.points = vector;
            obj.t_vec = obj.computeTimeIntervals();
            [obj.A, obj.c] = obj.computeMatrix();
            obj.v_vec = obj.computeVel();
            obj.m_coeffs = obj.computeCoeffs();
            [obj.p_seq,obj.v_seq,obj.a_seq,obj.j_seq] = obj.computeTraj();
            
        end
        
        function [A,c] = computeMatrix(obj)
            A = zeros((obj.length-2),(obj.length-2));
%             A((obj.length-2), (obj.length-2)) = 2*(obj.t_vec((obj.length-1))+obj.t_vec((obj.length)));
            for i=1:(obj.length-2)
                A(i,i) = 2*(obj.t_vec(i)+obj.t_vec(i+1));
                if (i<obj.length-3)
                    A(i+1,i) = obj.t_vec(i);
                    A(i,i+1) = obj.t_vec(i+2);
                end
            end
            c = zeros(obj.length-2, 1);
            for i=1:(obj.length-2)
                c(i) = 3/(obj.t_vec(i)*obj.t_vec(i+1))*(obj.t_vec(i)^2*(obj.points(i+2)-obj.points(i+1))+...
                        obj.t_vec(i+1)^2*(obj.t_vec(i+1)^2*(obj.points(i+1)-obj.points(i))));
            end
        end
        
        function v = computeVel(obj)
           v = [0; obj.A\obj.c; 0]; 
        end
        
        function coeff_m = computeCoeffs(obj)
           coeff_m = zeros(obj.length-1, 4);
           for i=1:obj.length-1
              a0 = obj.points(i);
              a1 = obj.v_vec(i);
              a2 = 1/obj.t_vec(i)*(3*(obj.points(i+1)-obj.points(i))/obj.t_vec(i) -2*obj.v_vec(i) - obj.v_vec(i+1));
              a3 = 1/obj.t_vec(i)^2*(2*(obj.points(i)-obj.points(i+1))/obj.t_vec(i) + obj.v_vec(i) + obj.v_vec(i+1));
              coeff_m(i,:) = [a0, a1, a2, a3];
           end
        end
        
        function [p,v,a,j] = computeTraj(obj)
            p = obj.points(1);
            v = 0;
            a = 0;
            j = 0;
            for i=1:obj.length-1
                for t=obj.delta_t:obj.delta_t:obj.t_vec(i)
                    a0 = obj.m_coeffs(i,1);
                    a1 = obj.m_coeffs(i,2);
                    a2 = obj.m_coeffs(i,3);
                    a3 = obj.m_coeffs(i,4);
                    point = a0 + a1*t + a2*t^2 + a3*t^3;
                    p = [p,  point];
                    v = [v, a1+2*a2*t+3*a3*t^2];
                    a = [a, 2*a2+6*a3*t];
                    j = [j, 6*a3];
                end
            end
        end
        
        function t = computeTimeIntervals(obj)
            for i=1:obj.length-1
                L(i) = abs(obj.points(i+1)-obj.points(i));
                t(i) = obj.T_tot/(obj.length-1);
            end
            L_tot = sum(L);
%             t = L/L_tot*obj.T_tot;
        end
    end
end

