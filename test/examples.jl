using DifferentialEquations, ModelingToolkit

###############################################################################
#                               Lead Lag example
# from: http://web.engr.oregonstate.edu/~webbky/ESE499_files/Section%209%20Root-Locus%20Design.pdf
# page 93
###############################################################################
function plant1(;name)
    @variables t y(t) dy(t) ddy(t) dddy(t) x(t)
    D = Differential(t)
    eqs = [
        D(y) ~ dy
        D(dy) ~ ddy
        D(ddy) ~ dddy
        15x ~ dddy+10*ddy+27*dy+18*y
    ]
    ODESystem(eqs, t;name, defaults=[x=>0.0, y=>0.0,
                                     dy=>0.0,ddy=>0.0, dddy=>0.0])
end

function ex_lead_lag()
    @variables t 
    @named p = plant1()
    @named lead = lead_lag_factory(k=130.0, zero=4.85, pole=100.0)
    @named lag = lead_lag_factory(k=1.0, zero=0.8, pole=0.08)
    connections = [lead.input ~ 1/(1+exp(-100*(t-1)))-p.y
                   lag.input ~ lead.output
                   p.x ~ lag.output]
    @named connected = ODESystem(connections ; systems=[p, lead, lag])
    sys = structural_simplify(connected)
    prob = ODEProblem(sys,[],(0.0,10.0))
    #plot(solve(prob),vars=[p.y])
end
# end lead lag example

###############################################################################
#                                 PID example
# from: https://ctms.engin.umich.edu/CTMS/index.php?example=Introduction&section=ControlPID
###############################################################################
function plant2(;name)
    @variables t F(t) x(t) dx(t)
    D = Differential(t)
    eqs = [
        D(x) ~ dx
        D(dx) ~ F - 10*dx - 20*x
    ]
    ODESystem(eqs, t;name, defaults=[x=>0.0, dx=>0.0])
end


function ex_pid()
    @variables t
    @named pid1 = PID_factory(kp=350.0,ki=300.0,kd=50.0)
    @named p = plant2()
    connections = [pid1.e ~ 1/(1+exp(-100*(t-1)))-p.x
                   p.F ~ pid1.u]
    @named connected = ODESystem(connections ; systems=[pid1,p])
    sys = structural_simplify(connected)
    prob = ODEProblem(sys,[],(0.0,10.0))
    return prob
end
# end PID example
