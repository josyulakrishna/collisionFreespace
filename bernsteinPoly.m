classdef bernsteinPoly
    
    properties
        X;
        Y;
        t0;
        tf;
        tc;
        degree;
        dt;
        wts;
    end
 
    methods
        
        function [ val ] = get_grad( obj, t, a, b, diff, mu, tdiff )
            
            if ( (a == 0) && (b == 0) )
                if (diff > 0)
                    val = 0;
                else
                    val = 1;
                end
            elseif ( a == 0 )
                if (diff > 0)
                    val = (b/tdiff)*get_grad(obj, t, 0, b-1, diff-1, mu, tdiff);
                else
                    val = (mu)^b;
                end
            elseif ( b == 0 )
                if (diff > 0)
                    val = -(a/tdiff)*get_grad(obj, t, a-1, 0, diff-1, mu, tdiff);
                else
                    val = (1-mu)^a;
                end
            else
                if (diff > 0)
                    val = (b/tdiff)*get_grad(obj, t, a, b-1, diff-1, mu, tdiff) - ...
                          (a/tdiff)*get_grad(obj, t, a-1, b, diff-1, mu, tdiff);
                else
                    val = (1-mu)^a*(mu)^b;
                end
            end
            
        end
        
        function [ poly ] = get_coefficients( obj, t, ndiff )
            n = obj.degree;
            d = ndiff;
            poly = zeros(n+1,1);
            mu = (t-obj.t0)/(obj.tf-obj.t0);
            tdiff = (obj.tf-obj.t0);
            for i = 0:n
                poly(i+1,1) = nchoosek(n,i)*(obj.get_grad(t, n-i, i, d, mu, tdiff));
            end
        end
        
%         function Wx = computeBernsteinWeights_x(obj)
%             
%             A = [];
%             B = [];
%             i = 2;
%             t_waypts = obj.tc;
%             for t = t_waypts(2:end-1)
%                 [ poly ] = obj.get_coefficients(t, 0);
%                 B(end+1,:)=[poly(2:end-1)];
%                 A(end+1,:)=obj.X(i) - obj.X(1)*poly(1) - obj.X(end)*poly(end);
%                 i=i+1;
%             end
%             Wx = pinv(B)*A; 
%             Wx = [obj.X(1); Wx; obj.X(end)]; 
%         end
%         
%         function Wy = computeBernsteinWeights_y(obj)
%             
%             A = [];
%             B = [];
%             i = 2;
%             t_waypts = obj.tc;
%             for t = t_waypts(2:end-1)
%                 [ poly ] = obj.get_coefficients(t, 0);
%                 B(end+1,:)=[poly(2:end-1)];
%                 A(end+1,:)=obj.Y(i) - obj.Y(1)*poly(1) - obj.Y(end)*poly(end);
%                 i=i+1; 
%             end
%             Wy = pinv(B)*A; 
%             Wy = [obj.Y(1); Wy; obj.Y(end)]; 
%         end
        
        function [x, y] = getBernsteinPoly(obj, delta_t)
            
            Wx = obj.wts(1:obj.degree+1);
            Wy = obj.wts(obj.degree+2:end);
            x = [];
            y = [];
            for t = obj.t0:delta_t:obj.tf
              [ poly ] = obj.get_coefficients(t, 0);
              x(end+1) = dot(poly, Wx);
              y(end+1) = dot(poly, Wy);
            end
        end
        
        function [velx,vely] = getBernsteinVel(obj, delta_t)

            Wx = obj.wts(1:obj.degree+1);
            Wy = obj.wts(obj.degree+2:end);
            velx = [];
            vely = [];
            for t=obj.t0:delta_t:obj.tf
              [ poly ] = obj.get_coefficients(t, 1);
              velx(end+1) = dot(poly, Wx);
              vely(end+1) = dot(poly, Wy);
            end
            
        end
        
    end
end 