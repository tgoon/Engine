within ;
package CCHPSystem
  package Engine
    "A library to demonstrate models used in analyzing vehicle performance"
    package Chassis "A collection of chassis models and related components"
      extends Modelica.Icons.Library2;
      model GenericCar "Generic car assembly"
        extends Interfaces.Chassis;
        parameter Modelica.SIunits.Mass vehicle_mass=1200 "Vehicle curb weight";
        parameter Real final_drive_ratio=3.43 "Final drive ratio";
        //Types.KilometersPerHour kmh "Vehicle speed";
        CCHPSystem.Engine.Types.KilometersPerHour kmh;
        Modelica.Mechanics.Rotational.Components.IdealGear final_drive( final
            ratio=
              final_drive_ratio) annotation (Placement(transformation(
              origin={40,0},
              extent={{-10,-10},{10,10}},
              rotation=180)));

        //Modelica.Mechanics.Rotational.IdealGear final_drive(final ratio=
          //    final_drive_ratio) annotation (extent=[30, -10; 50, 10], rotation=180);
        //Chassis.Tire tire annotation (extent=[-60, -20; -20, 20]);
        CCHPSystem.Engine.Chassis.Tire tire               annotation (Placement(
              transformation(extent={{-60,-20},{-20,20}}, rotation=0)));

        Modelica.Mechanics.Translational.Components.Mass vehicle_inertia(
                                                                     final m=
              vehicle_mass) annotation (Placement(transformation(extent={{-20,
                  20},{20,60}}, rotation=0)));
      equation
        connect(tire.body_force, vehicle_inertia.flange_a) annotation (Line(
              points={{-40,20},{-40,40},{-20,40}}, color={0,255,0}));
        connect(tire.road_force, road) annotation (Line(points={{-40,-20},{
                -40,-100}}, color={0,255,0}));
        connect(final_drive.flange_b, tire.axle) annotation (Line(points={{30,
                1.22465e-015},{-20,0}}));
        connect(final_drive.flange_a, power) annotation (Line(points={{50,
                -1.22465e-015},{80,0}}));
        kmh = der(vehicle_inertia.flange_a.s)*3.6;
        speed.signal[1] = kmh;
        annotation (
          Icon(coordinateSystem(
              preserveAspectRatio=false,
              extent={{-100,-100},{100,100}},
              grid={2,2}), graphics={
              Polygon(
                points={{-62,-40},{80,-40},{74,-22},{58,-16},{36,-12},{18,6},
                    {-30,6},{-40,-12},{-72,-20},{-72,-36},{-62,-40}},
                lineColor={0,0,0},
                fillColor={160,160,164},
                fillPattern=FillPattern.Solid),
              Ellipse(
                extent={{-52,-28},{-28,-52}},
                lineColor={0,0,255},
                pattern=LinePattern.None,
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid),
              Ellipse(
                extent={{-50,-30},{-30,-50}},
                lineColor={0,0,0},
                fillColor={128,128,128},
                fillPattern=FillPattern.Solid),
              Ellipse(
                extent={{48,-28},{72,-52}},
                lineColor={0,0,255},
                pattern=LinePattern.None,
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid),
              Ellipse(
                extent={{50,-30},{70,-50}},
                lineColor={0,0,0},
                fillColor={128,128,128},
                fillPattern=FillPattern.Solid),
              Polygon(
                points={{16,2},{30,-12},{10,-12},{-4,-12},{-4,2},{16,2}},
                lineColor={0,0,0},
                fillColor={192,192,192},
                fillPattern=FillPattern.Solid),
              Rectangle(
                extent={{-74,-30},{-64,-36}},
                lineColor={0,0,0},
                fillColor={0,0,0},
                fillPattern=FillPattern.Solid),
              Ellipse(
                extent={{54,-34},{66,-46}},
                lineColor={0,0,0},
                fillColor={192,192,192},
                fillPattern=FillPattern.Solid),
              Ellipse(
                extent={{-46,-34},{-34,-46}},
                lineColor={0,0,0},
                fillColor={192,192,192},
                fillPattern=FillPattern.Solid),
              Polygon(
                points={{-12,2},{-12,-12},{-34,-12},{-28,0},{-26,2},{-20,2},{
                    -12,2}},
                lineColor={0,0,0},
                fillColor={192,192,192},
                fillPattern=FillPattern.Solid),
              Polygon(points={{-6,4},{-6,-32},{24,-32},{32,-26},{32,-12},{16,
                    4},{-6,4}}, lineColor={128,128,128}),
              Line(points={{-40,-40},{-40,-100}}, color={127,255,0}),
              Text(extent={{-60,40},{60,20}}, textString=
                                                  "%name")}),
          Window(
            x=0.41,
            y=0.19,
            width=0.31,
            height=0.63),
          Diagram(coordinateSystem(
              preserveAspectRatio=false,
              extent={{-100,-100},{100,100}},
              grid={2,2}), graphics),
          Documentation(info="This model contains the primitives necessary to
model the chassis of a car.  This is a very simple model
with only a single inertia and not compliances in the
car frame.
"));
      end GenericCar;

      model Tire "Tire model"
        parameter Modelica.SIunits.Radius tire_radius=.35 "Tire radius";
      protected
        Modelica.SIunits.Position s_rel
          "Relative speed between chassis and road";

      public
        Modelica.Mechanics.Translational.Interfaces.Flange_a road_force 
          annotation (Placement(transformation(extent={{-10,-110},{10,-90}},
                rotation=0)));
        Modelica.Mechanics.Rotational.Interfaces.Flange_a axle annotation (Placement(
              transformation(extent={{90,-10},{110,10}}, rotation=0)));
        Modelica.Mechanics.Translational.Interfaces.Flange_b body_force 
          annotation (                           layer="icon", Placement(
              transformation(extent={{-10,90},{10,110}}, rotation=0)));
      equation
        road_force.f = axle.tau/tire_radius;
        road_force.f + body_force.f = 0;
        //  road_force.s = body_force.s;
        s_rel = axle.phi*tire_radius;
        body_force.s = road_force.s + s_rel;
        annotation (
          Icon(coordinateSystem(
              preserveAspectRatio=false,
              extent={{-100,-100},{100,100}},
              grid={2,2}), graphics={
              Rectangle(
                extent={{90,10},{54,-10}},
                lineColor={0,0,0},
                fillPattern=FillPattern.HorizontalCylinder,
                fillColor={192,192,192}),
              Ellipse(
                extent={{-60,60},{60,-60}},
                lineColor={0,0,0},
                lineThickness=0.5,
                fillColor={160,160,164},
                fillPattern=FillPattern.Solid),
              Ellipse(
                extent={{-40,40},{40,-40}},
                lineColor={0,0,0},
                fillColor={192,192,192},
                fillPattern=FillPattern.Solid)}),
          Window(
            x=0.51,
            y=0.23,
            width=0.37,
            height=0.71),
          Documentation(info="This tire model connects to a driven axle as well as the
vehicle chassis and the road.  The rotational velocity
of the axle is used to compute the relative translational
velocity between the road and the vehicle chassis based on
the tire radius.
"));
      end Tire;

      model Road
        Modelica.Mechanics.Translational.Interfaces.Flange_a road_surface 
          annotation (Placement(transformation(extent={{-10,90},{10,110}},
                rotation=0)));
      equation
        road_surface.s = 0;
        annotation (
          Icon(coordinateSystem(
              preserveAspectRatio=false,
              extent={{-100,-100},{100,100}},
              grid={2,2}), graphics={
              Rectangle(
                extent={{-100,60},{100,-60}},
                lineColor={160,160,164},
                fillColor={160,160,164},
                fillPattern=FillPattern.Solid),
              Rectangle(
                extent={{-80,4},{-40,-6}},
                lineColor={255,255,0},
                pattern=LinePattern.None,
                fillColor={255,255,0},
                fillPattern=FillPattern.Solid),
              Rectangle(
                extent={{-20,4},{20,-6}},
                lineColor={0,0,255},
                pattern=LinePattern.None,
                fillColor={255,255,0},
                fillPattern=FillPattern.Solid),
              Rectangle(
                extent={{40,4},{80,-6}},
                lineColor={0,0,255},
                pattern=LinePattern.None,
                fillColor={255,255,0},
                fillPattern=FillPattern.Solid),
              Rectangle(
                extent={{-100,54},{100,46}},
                lineColor={0,0,255},
                pattern=LinePattern.None,
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid),
              Rectangle(
                extent={{-100,-48},{100,-56}},
                lineColor={0,0,255},
                pattern=LinePattern.None,
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid),
              Line(points={{0,100},{0,60}}, color={0,255,0}),
              Text(extent={{-60,-60},{60,-100}}, textString=
                                                     "%name")}),
          Window(
            x=0.39,
            y=0.24,
            width=0.6,
            height=0.6));
      end Road;

      model SportsCarChassis "Sports Car Chassis"
        extends Chassis.GenericCar(vehicle_mass=1250, final_drive_ratio=3.45);
      equation
        connect(final_drive.flange_b, wheel) annotation (Line(points={{30,
                1.22465e-015},{30,-60},{-20,-60}}));
        annotation (
          Window(
            x=0.39,
            y=0.34,
            width=0.6,
            height=0.6),
          Documentation(info="This model extends the generic car chassis with the physical
characteristics of the Ford Mustang.
"));
      end SportsCarChassis;
      annotation (
        Window(
          x=0.46,
          y=0.03,
          width=0.28,
          height=0.42,
          library=1,
          autolayout=1),
        Documentation(info="This package contains all the chassis models and chassis component models
for the 'SimpleCar' package.  Some of the physical characteristics in this
package were found in the Bosch Automotive Handbook, 3rd Edition.
"),     Icon(coordinateSystem(
            preserveAspectRatio=false,
            extent={{0,0},{434,373}},
            grid={1,1}), graphics={
            Ellipse(
              extent={{-55,-36},{-35,-56}},
              lineColor={0,0,0},
              fillColor={128,128,128},
              fillPattern=FillPattern.Solid),
            Ellipse(
              extent={{-51,-40},{-39,-52}},
              lineColor={0,0,0},
              fillColor={192,192,192},
              fillPattern=FillPattern.Solid),
            Ellipse(
              extent={{44,-35},{64,-55}},
              lineColor={0,0,0},
              fillColor={128,128,128},
              fillPattern=FillPattern.Solid),
            Ellipse(
              extent={{48,-39},{60,-51}},
              lineColor={0,0,0},
              fillColor={192,192,192},
              fillPattern=FillPattern.Solid),
            Line(points={{-68,-67},{-45,-46}}, color={0,0,0}),
            Line(points={{31,-66},{54,-45}}, color={0,0,0}),
            Line(
              points={{-48,-50},{51,-50},{35,-63},{-64,-63},{-48,-50}},
              color={0,0,0},
              thickness=0.5),
            Ellipse(
              extent={{-80,-57},{-60,-77}},
              lineColor={0,0,0},
              fillColor={128,128,128},
              fillPattern=FillPattern.Solid),
            Ellipse(
              extent={{-75,-61},{-63,-73}},
              lineColor={0,0,0},
              fillColor={192,192,192},
              fillPattern=FillPattern.Solid),
            Ellipse(
              extent={{21,-57},{41,-77}},
              lineColor={0,0,0},
              fillColor={128,128,128},
              fillPattern=FillPattern.Solid),
            Ellipse(
              extent={{25,-61},{37,-73}},
              lineColor={0,0,0},
              fillColor={192,192,192},
              fillPattern=FillPattern.Solid)}));
    end Chassis;

    package Engine "Engine models and components"
      model BiogasPropertySource "Engine biogas property specification"

        parameter Modelica.SIunits.SpecificEnergy lhv "Lower heating value";
        parameter Real afr "Stoichiometric Air/Fuel Ratio";
        parameter Real SGF "Specific Gravity of Fuel";

        Interfaces.BiogasPropertyProvided bioP annotation (Placement(
              transformation(extent={{-114,-20},{-94,0}}), iconTransformation(
                extent={{-120,-20},{-100,0}})));
      equation
        bioP.lhv=lhv;
        bioP.SGF=SGF;
        bioP.afr=afr;
        annotation (
          Icon(coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},{
                  100,100}}), graphics={
              Line(
                points={{2,84},{-54,68},{-46,28},{-76,20},{-92,-26},{-68,-58},{
                    -46,-74},{-32,-90},{-6,-66},{42,-88},{68,-80},{88,-68},{66,
                    -20},{90,-8},{90,20},{64,28},{54,90},{10,74},{2,84}},
                color={0,0,255},
                smooth=Smooth.Bezier),
              Ellipse(
                extent={{18,-46},{24,-52}},
                lineColor={0,0,255},
                fillPattern=FillPattern.Solid,
                fillColor={85,255,255}),
              Ellipse(
                extent={{32,-46},{38,-52}},
                lineColor={0,0,255},
                fillPattern=FillPattern.Solid,
                fillColor={85,255,255}),
              Ellipse(
                extent={{34,-30},{40,-36}},
                lineColor={0,0,255},
                fillPattern=FillPattern.Solid,
                fillColor={85,255,255}),
              Ellipse(
                extent={{20,-32},{36,-48}},
                lineColor={0,0,255},
                fillPattern=FillPattern.Solid,
                fillColor={0,0,0}),
              Ellipse(
                extent={{-26,-40},{-20,-46}},
                lineColor={0,0,255},
                fillPattern=FillPattern.Solid,
                fillColor={255,0,0}),
              Ellipse(
                extent={{16,-30},{22,-36}},
                lineColor={0,0,255},
                fillPattern=FillPattern.Solid,
                fillColor={85,255,255}),
              Ellipse(
                extent={{-42,-34},{-26,-50}},
                lineColor={0,0,255},
                fillPattern=FillPattern.Solid,
                fillColor={0,0,0}),
              Ellipse(
                extent={{62,-64},{68,-70}},
                lineColor={0,0,255},
                fillPattern=FillPattern.Solid,
                fillColor={255,0,0}),
              Ellipse(
                extent={{46,-58},{62,-74}},
                lineColor={0,0,255},
                fillPattern=FillPattern.Solid,
                fillColor={0,0,0}),
              Ellipse(
                extent={{54,-52},{60,-58}},
                lineColor={0,0,255},
                fillPattern=FillPattern.Solid,
                fillColor={255,0,0}),
              Ellipse(
                extent={{30,52},{36,46}},
                lineColor={0,0,255},
                fillPattern=FillPattern.Solid,
                fillColor={255,0,0}),
              Ellipse(
                extent={{14,58},{30,42}},
                lineColor={0,0,255},
                fillPattern=FillPattern.Solid,
                fillColor={0,0,0}),
              Ellipse(
                extent={{22,64},{28,58}},
                lineColor={0,0,255},
                fillPattern=FillPattern.Solid,
                fillColor={255,0,0}),
              Ellipse(
                extent={{-38,30},{-32,24}},
                lineColor={0,0,255},
                fillPattern=FillPattern.Solid,
                fillColor={85,255,255}),
              Ellipse(
                extent={{-24,30},{-18,24}},
                lineColor={0,0,255},
                fillPattern=FillPattern.Solid,
                fillColor={85,255,255}),
              Ellipse(
                extent={{-22,46},{-16,40}},
                lineColor={0,0,255},
                fillPattern=FillPattern.Solid,
                fillColor={85,255,255}),
              Ellipse(
                extent={{-36,44},{-20,28}},
                lineColor={0,0,255},
                fillPattern=FillPattern.Solid,
                fillColor={0,0,0}),
              Ellipse(
                extent={{-40,46},{-34,40}},
                lineColor={0,0,255},
                fillPattern=FillPattern.Solid,
                fillColor={85,255,255}),
              Ellipse(
                extent={{-62,-10},{-56,-16}},
                lineColor={0,0,255},
                fillPattern=FillPattern.Solid,
                fillColor={85,255,255}),
              Ellipse(
                extent={{-48,-10},{-42,-16}},
                lineColor={0,0,255},
                fillPattern=FillPattern.Solid,
                fillColor={85,255,255}),
              Ellipse(
                extent={{-46,6},{-40,0}},
                lineColor={0,0,255},
                fillPattern=FillPattern.Solid,
                fillColor={85,255,255}),
              Ellipse(
                extent={{-60,4},{-44,-12}},
                lineColor={0,0,255},
                fillPattern=FillPattern.Solid,
                fillColor={0,0,0}),
              Ellipse(
                extent={{-64,6},{-58,0}},
                lineColor={0,0,255},
                fillPattern=FillPattern.Solid,
                fillColor={85,255,255}),
              Ellipse(
                extent={{-2,0},{4,-6}},
                lineColor={0,0,255},
                fillPattern=FillPattern.Solid,
                fillColor={85,255,255}),
              Ellipse(
                extent={{12,0},{18,-6}},
                lineColor={0,0,255},
                fillPattern=FillPattern.Solid,
                fillColor={85,255,255}),
              Ellipse(
                extent={{14,16},{20,10}},
                lineColor={0,0,255},
                fillPattern=FillPattern.Solid,
                fillColor={85,255,255}),
              Ellipse(
                extent={{0,14},{16,-2}},
                lineColor={0,0,255},
                fillPattern=FillPattern.Solid,
                fillColor={0,0,0}),
              Ellipse(
                extent={{-4,16},{2,10}},
                lineColor={0,0,255},
                fillPattern=FillPattern.Solid,
                fillColor={85,255,255})}),
          Coordsys(
            extent=[-100, -100; 100, 100],
            grid=[2, 2],
            component=[20, 20]),
          Window(
            x=0.45,
            y=0.16,
            width=0.6,
            height=0.6),
          Documentation(info="This model takes basic engine geometry parameters and computes the complete
set of engine geometry characteristics.
"),       Diagram(coordinateSystem(preserveAspectRatio=true,  extent={{-100,
                  -100},{100,100}}),
                              graphics));
      end BiogasPropertySource;
      extends Modelica.Icons.Library2;

      package GasProperties "Gas property related definitions"
        partial model BasePropertyModel "Basic property model interface"
          Modelica.SIunits.Pressure P "Gas pressure";
          Modelica.SIunits.Temperature T "Gas temperature";
          Modelica.SIunits.SpecificEnthalpy h "Specific enthalpy of gas";
          Modelica.SIunits.SpecificEnergy u "Specific energy of gas";
          Real gamma "ratio of specific heats";
          Modelica.SIunits.MolarMass mw "Molecular weight of gas";
          Modelica.SIunits.Density ro;
          annotation (
            Window(
              x=0.39,
              y=0.34,
              width=0.6,
              height=0.6),
            Documentation(info="This interface defines the relationship between the state of a gas (pressure and temperature)
and the intensive properties of the gas (specific enthalpy, specific energy).  This is a generic
interface that should be implemented with four equations.  The equations will generally compute
the intensive properties of the gas directly from the state but it could be used \"in reverse\" to
compute pressure and temperature based on two intensive properties.
"));
        end BasePropertyModel;

        model SimpleAirProperties "Very simple air properties"
          extends BasePropertyModel;
          parameter Modelica.SIunits.SpecificHeatCapacity cp=1039
            "Specific heat capacity at constant pressure";
          parameter Modelica.SIunits.MolarMass mole_weight=0.028964
            "Molecular weight";
        protected
          parameter Modelica.SIunits.SpecificHeatCapacity cv=cp - (Modelica.
              Constants.R/mole_weight);
        parameter Modelica.SIunits.Density ro0= 1.293;
        equation
          h = cp*T;
          u = cv*T;
          mw = mole_weight;
          gamma = cp/cv;
          ro=ro0*(P/101325)*(273.15/T);
          annotation (
            Window(
              x=0.39,
              y=0.34,
              width=0.6,
              height=0.6),
            Documentation(info="This property model assumes the gas is a perfect gas with the properties of air at room temperature.
This is an overly simplified model of air but sufficient for demonstrating the basics of engine
thermodynamics.
"));
        end SimpleAirProperties;

        model SimpleGasProperty
          extends BasePropertyModel;
         parameter Modelica.SIunits.SpecificHeatCapacity cp=1039
            "Specific heat capacity at constant pressure";
          parameter Modelica.SIunits.MolarMass mole_weight=0.028964
            "Molecular weight";
          parameter Modelica.SIunits.SpecificHeatCapacity cv=cp - (Modelica.
              Constants.R/mole_weight);
         parameter Modelica.SIunits.Density ro0= 1.293;
        equation
          h = cp*T;
          u = cv*T;
          mw = mole_weight;
          gamma = cp/cv;
          ro=ro0*(P/101325)*(273.15/T);
        end SimpleGasProperty;
      end GasProperties;

      model GeometrySource "Engine geometry specification"
        parameter Modelica.SIunits.Length bore "Engine bore";
        parameter Modelica.SIunits.Length stroke "Engine stroke";
        parameter Modelica.SIunits.Length conrod "Connecting rod length";
        parameter Real comp_ratio=9.5 "Compression ratio";
        Interfaces.EngineGeometryProvided geom annotation (Placement(
              transformation(
              origin={-110,0},
              extent={{-10,-10},{10,10}},
              rotation=180)));
      equation
        assert(bore > 0, "Invalid bore value");
        assert(stroke > 0, "Invalid stroke value");
        assert(conrod > 0, "Invalid connecting rod length");
        assert(comp_ratio > 1, "Invalid compression ratio");
        geom.bore = bore;
        geom.stroke = stroke;
        geom.conrod = conrod;
        geom.Vc = geom.Vd/(comp_ratio - 1);
        geom.Ap = Modelica.Constants.pi*(bore/2)^2;
        geom.Vd = stroke*geom.Ap;
        geom.crank = stroke/2.0;
        annotation (
          Icon(coordinateSystem(
              preserveAspectRatio=false,
              extent={{-100,-100},{100,100}},
              grid={2,2}), graphics={
              Polygon(
                points={{-60,-100},{-60,40},{-42,40},{-42,44},{-40,46},{-60,
                    60},{-100,60},{-100,-100},{-60,-100}},
                lineColor={0,0,0},
                fillColor={192,192,192},
                fillPattern=FillPattern.Solid),
              Rectangle(
                extent={{-58,12},{58,-64}},
                lineColor={0,0,0},
                fillPattern=FillPattern.VerticalCylinder,
                fillColor={192,192,192}),
              Rectangle(
                extent={{-60,6},{60,0}},
                lineColor={0,0,0},
                fillColor={0,0,0},
                fillPattern=FillPattern.Solid),
              Rectangle(
                extent={{-60,-8},{60,-14}},
                lineColor={0,0,0},
                fillColor={0,0,0},
                fillPattern=FillPattern.Solid),
              Rectangle(
                extent={{-60,-20},{60,-26}},
                lineColor={0,0,0},
                fillColor={0,0,0},
                fillPattern=FillPattern.Solid),
              Polygon(
                points={{-58,-64},{-40,-52},{40,-52},{58,-64},{-58,-64}},
                lineColor={0,0,255},
                pattern=LinePattern.None,
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid),
              Ellipse(
                extent={{-4,-34},{4,-42}},
                lineColor={0,0,0},
                fillColor={0,0,0},
                fillPattern=FillPattern.Solid),
              Rectangle(
                extent={{-42,44},{-18,40}},
                lineColor={0,0,0},
                fillPattern=FillPattern.VerticalCylinder,
                fillColor={192,192,192}),
              Polygon(
                points={{-42,44},{-40,46},{-20,46},{-18,44},{-42,44}},
                lineColor={160,160,164},
                fillColor={192,192,192},
                fillPattern=FillPattern.Solid),
              Rectangle(
                extent={{-32,46},{-28,94}},
                lineColor={0,0,0},
                fillPattern=FillPattern.VerticalCylinder,
                fillColor={192,192,192}),
              Polygon(
                points={{18,44},{20,46},{40,46},{42,44},{18,44}},
                lineColor={160,160,164},
                fillColor={192,192,192},
                fillPattern=FillPattern.Solid),
              Rectangle(
                extent={{18,44},{42,40}},
                lineColor={0,0,0},
                fillPattern=FillPattern.VerticalCylinder,
                fillColor={192,192,192}),
              Rectangle(
                extent={{28,46},{32,96}},
                lineColor={0,0,0},
                fillPattern=FillPattern.VerticalCylinder,
                fillColor={192,192,192}),
              Polygon(
                points={{100,-100},{100,60},{60,60},{40,46},{42,44},{42,40},{
                    60,40},{60,-100},{100,-100}},
                lineColor={0,0,0},
                fillColor={192,192,192},
                fillPattern=FillPattern.Solid),
              Polygon(
                points={{-100,96},{-60,96},{-40,80},{-20,60},{-20,46},{-18,44},
                    {-18,40},{18,40},{18,44},{20,46},{20,60},{40,80},{60,96},
                    {100,96},{100,100},{-100,100},{-100,96}},
                lineColor={0,0,0},
                fillColor={192,192,192},
                fillPattern=FillPattern.Solid),
              Text(extent={{-60,-66},{60,-100}}, textString=
                                                     "%name")}),
          Window(
            x=0.45,
            y=0.16,
            width=0.6,
            height=0.6),
          Documentation(info="This model takes basic engine geometry parameters and computes the complete
set of engine geometry characteristics.
"),       Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,
                  -100},{100,100}},
              grid={2,2}),    graphics));
      end GeometrySource;

      model SportsCarGeometry "Geometry of a sports car with an I4 engine"
        extends GeometrySource(
          bore=.09604,
          stroke=.0794,
          comp_ratio=9.5,
          conrod=0.157);
        annotation (
          Window(
            x=0.1,
            y=0.1,
            width=0.6,
            height=0.6),
          Documentation(info="This model has default engine geometry parameters that would be typical of a sports car.
"));
      end SportsCarGeometry;

      package Components "A collection of components used to build engines"
        extends Modelica.Icons.Library2;

        replaceable model PropertyModel = GasProperties.SimpleAirProperties;
        model ChamberVolume
          "Computes combustion chamber volume as a function of piston position"

          Modelica.Mechanics.Translational.Interfaces.Flange_a piston annotation (Placement(
                transformation(extent={{-10,-90},{10,-70}}, rotation=0)));
          Modelica.Blocks.Interfaces.RealOutput volume         annotation (Placement(
                transformation(
                origin={-110,0},
                extent={{-10,-10},{10,10}},
                rotation=180)));
          Interfaces.EngineGeometryRequired geom annotation (Placement(
                transformation(
                origin={110,0},
                extent={{-10,-10},{10,10}},
                rotation=180)));
        equation
          volume = geom.Vc + geom.Ap*piston.s;
          piston.f = 0;
          annotation (
            Window(
              x=0.54,
              y=0.18,
              width=0.39,
              height=0.79),
            Icon(coordinateSystem(
                preserveAspectRatio=false,
                extent={{-100,-100},{100,100}},
                grid={2,2}), graphics={
                Polygon(
                  points={{-60,-100},{-60,40},{-42,40},{-42,44},{-40,46},{-60,
                      60},{-100,60},{-100,-100},{-60,-100}},
                  lineColor={0,0,0},
                  fillColor={255,255,255},
                  fillPattern=FillPattern.Forward),
                Polygon(
                  points={{-100,96},{-60,96},{-40,80},{-20,60},{-20,46},{-18,
                      44},{-18,40},{18,40},{18,44},{20,46},{20,60},{40,80},{
                      60,96},{100,96},{100,100},{-100,100},{-100,96}},
                  lineColor={0,0,0},
                  fillColor={255,255,255},
                  fillPattern=FillPattern.Forward),
                Rectangle(
                  extent={{-58,12},{58,-64}},
                  lineColor={0,0,0},
                  fillPattern=FillPattern.VerticalCylinder,
                  fillColor={192,192,192}),
                Rectangle(
                  extent={{-60,6},{60,0}},
                  lineColor={0,0,0},
                  fillColor={0,0,0},
                  fillPattern=FillPattern.Solid),
                Rectangle(
                  extent={{-60,-8},{60,-14}},
                  lineColor={0,0,0},
                  fillColor={0,0,0},
                  fillPattern=FillPattern.Solid),
                Rectangle(
                  extent={{-60,-20},{60,-26}},
                  lineColor={0,0,0},
                  fillColor={0,0,0},
                  fillPattern=FillPattern.Solid),
                Polygon(
                  points={{-58,-64},{-40,-52},{40,-52},{58,-64},{-58,-64}},
                  lineColor={0,0,255},
                  pattern=LinePattern.None,
                  fillColor={255,255,255},
                  fillPattern=FillPattern.Solid),
                Ellipse(
                  extent={{-4,-34},{4,-42}},
                  lineColor={0,0,0},
                  fillColor={0,0,0},
                  fillPattern=FillPattern.Solid),
                Rectangle(
                  extent={{-42,36},{-18,32}},
                  lineColor={0,0,0},
                  fillPattern=FillPattern.VerticalCylinder,
                  fillColor={192,192,192}),
                Polygon(
                  points={{-42,36},{-40,38},{-20,38},{-18,36},{-42,36}},
                  lineColor={160,160,164},
                  fillColor={192,192,192},
                  fillPattern=FillPattern.Solid),
                Rectangle(
                  extent={{-32,38},{-28,96}},
                  lineColor={0,0,0},
                  fillPattern=FillPattern.VerticalCylinder,
                  fillColor={192,192,192}),
                Polygon(
                  points={{18,44},{20,46},{40,46},{42,44},{18,44}},
                  lineColor={160,160,164},
                  fillColor={192,192,192},
                  fillPattern=FillPattern.Solid),
                Rectangle(
                  extent={{18,44},{42,40}},
                  lineColor={0,0,0},
                  fillPattern=FillPattern.VerticalCylinder,
                  fillColor={192,192,192}),
                Rectangle(
                  extent={{28,46},{32,96}},
                  lineColor={0,0,0},
                  fillPattern=FillPattern.VerticalCylinder,
                  fillColor={192,192,192}),
                Polygon(
                  points={{100,-100},{100,60},{60,60},{40,46},{42,44},{42,40},
                      {60,40},{60,-100},{100,-100}},
                  lineColor={0,0,0},
                  fillColor={255,255,255},
                  fillPattern=FillPattern.Forward),
                Text(extent={{-58,126},{60,94}}, textString=
                                                     "%name")}),
            Documentation(info="This model is used to compute the combustion chamber volume as a function
of piston position.  The has connectors for providing engine geometry
characteristics and the piston position and computes the chamber volume
as an output signal.

Note that this model does not really interact with the piston because it
does not result in any force being applied to the piston.  It is important
when creating models that are \"read-only\" for a given domain (e.g. the 
translational domain in the case of the piston position) that a zero
contribution be provided for any flow variables.
"),         Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,
                    -100},{100,100}},
                grid={2,2}),           graphics));
        end ChamberVolume;

        model ControlVolume "Thermodynamic control volume"
        protected
          Modelica.SIunits.Energy U(start=147) "Total energy";
          Modelica.SIunits.Mass m(start=4e-4) "Total mass";
          Modelica.SIunits.Pressure P(start=101800) = state.P "Pressure";
          Modelica.SIunits.Temperature T(start=300) = state.T "Temperature";
          Modelica.SIunits.MassFlowRate mdot=state.mdot "Net mass flow";
          Modelica.SIunits.Power q=state.q "Net power";
          Modelica.SIunits.Volume V=volume "Chamber volume";
          Modelica.SIunits.AmountOfSubstance N "Number of moles of gas";
          Real R=Modelica.Constants.R;
          Real logV;
          Real logP;
        public
          Modelica.Blocks.Interfaces.RealInput volume         annotation (Placement(
                transformation(
                origin={110,0},
                extent={{-10,-10},{10,10}},
                rotation=180)));
        public
          Modelica.Blocks.Interfaces.RealOutput mass 
                                                  annotation (Placement(
                transformation(
                origin={-60,110},
                extent={{-10,-10},{10,10}},
                rotation=90)));
        public
          Interfaces.Gas state "Gas state" annotation (
              layer="icon", Placement(transformation(extent={{-10,-10},{10,10}},
                  rotation=0)));
        protected
          PropertyModel props(T=T, P=P) annotation (Placement(transformation(
                  extent={{-80,40},{-40,80}}, rotation=0)));
        equation
          // Compute number of moles
          N = m/props.mw;

          // Ideal gas low
          P*V = N*R*T;

          // First law of thermodynamics
          der(U) = q - P*der(V);
          U = m*props.u;

          // Conservation of mass
          der(m) = mdot;
          mass = m;
          logV = Modelica.Math.log(V);
          logP = Modelica.Math.log(abs(P));
          annotation (
            Icon(coordinateSystem(
                preserveAspectRatio=false,
                extent={{-100,-100},{100,100}},
                grid={2,2}), graphics={
                Rectangle(extent={{-60,60},{60,-60}}, lineColor={0,0,0}),
                Ellipse(
                  extent={{14,46},{18,50}},
                  lineColor={0,127,255},
                  fillColor={0,127,255},
                  fillPattern=FillPattern.Solid),
                Line(
                  points={{16,48},{4,60},{-18,42}},
                  color={0,0,0},
                  pattern=LinePattern.Dot),
                Ellipse(
                  extent={{52,20},{56,24}},
                  lineColor={0,127,255},
                  fillColor={0,127,255},
                  fillPattern=FillPattern.Solid),
                Ellipse(
                  extent={{-54,18},{-50,22}},
                  lineColor={0,127,255},
                  fillColor={0,127,255},
                  fillPattern=FillPattern.Solid),
                Line(
                  points={{54,22},{60,14},{52,2}},
                  color={0,0,0},
                  pattern=LinePattern.Dot),
                Line(
                  points={{-54,40},{-60,32},{-52,20}},
                  color={0,0,0},
                  pattern=LinePattern.Dot),
                Ellipse(
                  extent={{0,18},{4,14}},
                  lineColor={0,127,255},
                  fillColor={0,127,255},
                  fillPattern=FillPattern.Solid),
                Line(
                  points={{2,16},{-18,10}},
                  color={0,0,0},
                  pattern=LinePattern.Dot),
                Ellipse(
                  extent={{-40,-42},{-36,-46}},
                  lineColor={0,127,255},
                  fillColor={0,127,255},
                  fillPattern=FillPattern.Solid),
                Line(
                  points={{-38,-44},{-34,-60},{-30,-34}},
                  color={0,0,0},
                  pattern=LinePattern.Dot),
                Ellipse(
                  extent={{24,-24},{28,-28}},
                  lineColor={0,127,255},
                  pattern=LinePattern.Dot,
                  fillColor={0,127,255},
                  fillPattern=FillPattern.Solid),
                Line(
                  points={{26,-26},{38,-52}},
                  color={0,0,0},
                  pattern=LinePattern.Dot),
                Text(extent={{-50,108},{80,80}}, textString=
                                                     "%name"),
                Polygon(
                  points={{-80,68},{-80,80},{-60,60},{-60,-60},{60,-60},{60,
                      60},{-60,60},{-80,80},{80,80},{80,-80},{-80,-80},{-80,
                      68}},
                  lineColor={0,0,0},
                  fillColor={255,255,255},
                  fillPattern=FillPattern.Backward)}),
            Window(
              x=0.32,
              y=0.32,
              width=0.6,
              height=0.6),
            Documentation(info="This model contains the basic equations of a thermodynamic control volume.  These include the conservation of mass and energy
as well as the ideal gas law and gas property equations.  The volume of the control volume is an input to this model
and the current mass contained within the control volume is an output.  The output signal for mass is necessary to
connect a combustion model.
"),         Diagram(coordinateSystem(preserveAspectRatio=false,extent={{-100,
                    -100},{100,100}},
                grid={2,2}),           graphics));
        end ControlVolume;

        model Orifice "Isentropic flow restriction"
          parameter Modelica.SIunits.Area Aref "Reference Area";
        protected
          Modelica.SIunits.MassFlowRate mdot "Flow from 'a' to 'b'";
          Modelica.SIunits.SpecificEnthalpy h "Upstream enthalpy";
          Modelica.SIunits.SpecificEnthalpy gamma "Upstream gamma";
          Real pratio "Pressure ratio";
          Real Cd "Discharge Coefficient";
        public
          Interfaces.Gas a annotation (Placement(transformation(extent={{-110,
                    -10},{-90,10}}, rotation=0)));
          Interfaces.Gas b annotation (Placement(transformation(extent={{-10,
                    -110},{10,-90}}, rotation=0)));
        protected
          PropertyModel a_props(T=a.T, P=a.P) annotation (Placement(
                transformation(extent={{-56.6667,10},{-10,56.6667}}, rotation=
                   0)));
          PropertyModel b_props(T=b.T, P=b.P) annotation (Placement(
                transformation(extent={{10,10},{56.6667,56.6667}}, rotation=0)));
        equation
          a.mdot = mdot;
          b.mdot = -mdot;
          a.q = mdot*h;
          b.q = -mdot*h;
         if (a.P > b.P) then
            h = a_props.h;
            gamma = a_props.gamma;
            pratio = b.P/a.P;
            if (pratio <= (2.0/(gamma + 1.0))^(gamma/(gamma - 1.0))) then
              mdot = Cd*Aref*a.P/((Modelica.Constants.R/a_props.mw)*a.T)^0.5*gamma^0.5*(2.0/(gamma + 1.0))^((gamma + 1.0)/(2.0*(gamma - 1.0)));
            else
            // mdot = Cd*Aref*a.P/((Modelica.Constants.R/b_props.mw)*b.T)^0.5*pratio^(1.0/gamma)*(2.0*gamma/(gamma - 1.0)*(1.0 - pratio^((gamma- 1.0)/gamma)))^0.5;
            mdot = Cd*Aref*a.P/((Modelica.Constants.R/a_props.mw)*a.T)^0.5*pratio^(1.0/gamma)*(max({0,(2.0*gamma/(gamma - 1.0)*(1.0 - pratio^((gamma- 1.0)/gamma)))}))^0.5;
            end if;

        else
            h = b_props.h;
            gamma = b_props.gamma;
            pratio = a.P/b.P;
            if (pratio <= (2.0/(gamma + 1.0))^(gamma/(gamma - 1.0))) then
              mdot = -Cd*Aref*b.P/((Modelica.Constants.R/b_props.mw)*b.T)^0.5*gamma^0.5*(2.0/(gamma + 1.0))^((gamma + 1.0)/(2.0*(gamma - 1.0)));
            else
              // mdot = -Cd*Aref*a.P/((Modelica.Constants.R/b_props.mw)*b.T)^0.5*pratio^(1.0/gamma)*(2.0*gamma/(gamma - 1.0)*(1.0 - pratio^((gamma- 1.0)/gamma)))^0.5;
            mdot = -Cd*Aref*b.P/((Modelica.Constants.R/a_props.mw)*a.T)^0.5*pratio^(1.0/gamma)*(max({0,(2.0*gamma/(gamma - 1.0)*(1.0 - pratio^((gamma- 1.0)/gamma)))}))^0.5;
          end if;
          end if;
          annotation (
            Window(
              x=0.23,
              y=0.19,
              width=0.6,
              height=0.6),
            Documentation(info="This is a base model for other models which model isentropic flow (e.g. throttles, engine valves)
"));
        end Orifice;

        model Throttle "Orifice with throttle plate"
          parameter Modelica.SIunits.Diameter dia=0.05 "Throttle diameter";
          extends Orifice(Aref=Modelica.Constants.pi*(dia/2)^2);//2011-11-8changes based on BioDes2
          Modelica.Blocks.Interfaces.RealInput throttle_angle
            "Throttle Angle [deg]" annotation (Placement(transformation(
                origin={0,110},
                extent={{-10,-10},{10,10}},
                rotation=270)));
        equation
          Cd = 0.1*Modelica.Math.sin(throttle_angle*Modelica.Constants.pi/180)^2;

          annotation (
            Window(
              x=0.16,
              y=0.29,
              width=0.31,
              height=0.63),
            Icon(coordinateSystem(
                preserveAspectRatio=false,
                extent={{-100,-100},{100,100}},
                grid={2,2}), graphics={
                Rectangle(
                  extent={{-60,80},{60,-80}},
                  lineColor={0,0,0},
                  fillPattern=FillPattern.VerticalCylinder,
                  fillColor={192,192,192}),
                Line(
                  points={{30,32},{-30,-28}},
                  color={0,0,0},
                  thickness=0.5),
                Ellipse(
                  extent={{-6,8},{6,-4}},
                  lineColor={0,0,0},
                  fillColor={0,0,0},
                  fillPattern=FillPattern.Solid),
                Text(extent={{20,100},{100,80}}, textString=
                                                     "%name")}),
            Documentation(info="A very simple engine throttle.  The input signal is the throttle angle.
"));
        end Throttle;

        model Valve "Engine poppet valve"
          parameter Modelica.SIunits.Diameter dia=0.012 "Valve diameter";
          parameter Modelica.SIunits.Length max_lift=0.012 "Maximum Valve Lift";
          parameter Real max_discharge=0.7 "Maximum Discharge Coefficient";
          extends Orifice(final Aref=Modelica.Constants.pi*(dia/2)^2);
        protected
          parameter Real c_over_l=max_discharge/max_lift;
        public
          Modelica.Mechanics.Translational.Interfaces.Flange_a lift annotation (
                                         layer="icon", Placement(
                transformation(extent={{-10,90},{10,110}}, rotation=0)));
        equation
          lift.f = 0;
          Cd = c_over_l*lift.s;
          // Cd = 0.0;
          annotation (
            Icon(coordinateSystem(
                preserveAspectRatio=false,
                extent={{-100,-100},{100,100}},
                grid={2,2}), graphics={
                Rectangle(
                  extent={{-40,-60},{40,-70}},
                  lineColor={0,0,0},
                  fillPattern=FillPattern.VerticalCylinder,
                  fillColor={192,192,192}),
                Polygon(
                  points={{-40,-60},{-28,-52},{30,-52},{40,-60},{-40,-60}},
                  lineColor={0,0,0},
                  fillPattern=FillPattern.VerticalCylinder,
                  fillColor={160,160,164}),
                Rectangle(
                  extent={{-10,90},{10,-52}},
                  lineColor={0,0,0},
                  fillPattern=FillPattern.VerticalCylinder,
                  fillColor={192,192,192}),
                Polygon(
                  points={{-40,-54},{-28,-46},{-28,-40},{-40,-30},{-60,-16},{
                      -100,-16},{-100,-100},{-60,-100},{-60,-70},{-60,-58},{
                      -40,-58},{-40,-54}},
                  lineColor={0,0,0},
                  fillColor={255,255,255},
                  fillPattern=FillPattern.Forward),
                Polygon(
                  points={{-100,40},{-60,40},{-10,16},{-10,100},{-100,100},{
                      -100,40}},
                  lineColor={0,0,0},
                  fillColor={255,255,255},
                  fillPattern=FillPattern.Forward),
                Polygon(
                  points={{10,6},{20,0},{30,-40},{30,-46},{40,-54},{40,-58},{
                      60,-58},{60,-100},{100,-100},{100,100},{10,100},{10,6}},
                  lineColor={0,0,0},
                  fillColor={255,255,255},
                  fillPattern=FillPattern.Forward),
                Text(extent={{12,64},{100,44}}, textString=
                                                    "%name")}),
            Window(
              x=0.39,
              y=0.31,
              width=0.6,
              height=0.6),
            Documentation(info="This is a very simple model of an engine valve the is derived from the \"Orifice\" model of isentropic flow.
The valve model must be connected to two different gas volumes (or reservoirs).  In addition, a translational
connector is used to represent the lift of the valve.
"));
        end Valve;

        model MasslessPiston "A massless piston"
          parameter Modelica.SIunits.Pressure Pcc=101800 "Crankcase pressure";
          Interfaces.EngineGeometryRequired geom annotation (Placement(
                transformation(
                origin={110,0},
                extent={{-10,-10},{10,10}},
                rotation=180)));
          Interfaces.Gas chamber annotation (Placement(transformation(extent=
                    {{-10,90},{10,110}}, rotation=0)));
          Modelica.Mechanics.Translational.Interfaces.Flange_a piston annotation (Placement(
                transformation(extent={{-10,-32},{10,-12}}, rotation=0)));
        equation
          piston.f = geom.Ap*(chamber.P - Pcc);
          chamber.mdot = 0;
          chamber.q = 0;

          annotation (
            Icon(coordinateSystem(
                preserveAspectRatio=false,
                extent={{-100,-100},{100,100}},
                grid={2,2}), graphics={
                Rectangle(
                  extent={{-94,90},{94,-100}},
                  lineColor={0,0,0},
                  fillPattern=FillPattern.VerticalCylinder,
                  fillColor={192,192,192}),
                Rectangle(
                  extent={{-100,80},{100,74}},
                  lineColor={0,0,0},
                  fillColor={0,0,0},
                  fillPattern=FillPattern.Solid),
                Rectangle(
                  extent={{-100,42},{100,36}},
                  lineColor={0,0,0},
                  fillColor={0,0,0},
                  fillPattern=FillPattern.Solid),
                Rectangle(
                  extent={{-100,60},{100,54}},
                  lineColor={0,0,0},
                  fillColor={0,0,0},
                  fillPattern=FillPattern.Solid),
                Polygon(
                  points={{-94,-100},{-60,-58},{60,-58},{94,-100},{-94,-100}},
                  lineColor={0,0,255},
                  pattern=LinePattern.None,
                  fillColor={255,255,255},
                  fillPattern=FillPattern.Solid),
                Ellipse(
                  extent={{-20,-2},{20,-42}},
                  lineColor={0,0,0},
                  fillColor={0,0,0},
                  fillPattern=FillPattern.Solid),
                Text(extent={{-40,-70},{42,-90}}, textString=
                                                      "%name")}),
            Window(
              x=0.39,
              y=0.22,
              width=0.6,
              height=0.6),
            Documentation(info="This piston is used to translate pressure inside the cylinder into force (presumably applied to the crank slider mechanism).
This piston has no mass.
"),         Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,
                    -100},{100,100}},
                grid={2,2}),           graphics));
        end MasslessPiston;

        model OffsetShaft "Angular displacement"
          parameter Types.Degrees shift=0 "Shift from crankshaft";
          Types.RPM shaft_speed;
          Modelica.Mechanics.Rotational.Interfaces.Flange_a crank annotation (Placement(
                transformation(extent={{-110,-10},{-90,10}}, rotation=0)));
          Modelica.Mechanics.Rotational.Interfaces.Flange_b cyl annotation (Placement(
                transformation(extent={{-10,-110},{10,-90}}, rotation=0)));
        equation
          shaft_speed = 30*der(crank.phi)/Modelica.Constants.pi;
          crank.tau + cyl.tau = 0;
          cyl.phi = crank.phi + shift*Modelica.Constants.pi/180.0;
          annotation (
            Icon(coordinateSystem(
                preserveAspectRatio=false,
                extent={{-100,-100},{100,100}},
                grid={2,2}), graphics={
                Rectangle(
                  extent={{-90,10},{-40,-10}},
                  lineColor={0,0,0},
                  fillPattern=FillPattern.HorizontalCylinder,
                  fillColor={192,192,192}),
                Rectangle(
                  extent={{-40,10},{-20,-80}},
                  lineColor={160,160,164},
                  fillColor={192,192,192},
                  fillPattern=FillPattern.Solid),
                Rectangle(
                  extent={{-20,-40},{20,-80}},
                  lineColor={0,0,0},
                  fillPattern=FillPattern.HorizontalCylinder,
                  fillColor={192,192,192}),
                Rectangle(
                  extent={{20,10},{40,-80}},
                  lineColor={160,160,164},
                  fillColor={192,192,192},
                  fillPattern=FillPattern.Solid),
                Rectangle(
                  extent={{40,10},{90,-10}},
                  lineColor={0,0,0},
                  fillPattern=FillPattern.HorizontalCylinder,
                  fillColor={192,192,192}),
                Text(extent={{-40,40},{40,20}}, textString=
                                                    "%name")}),
            Window(
              x=0.39,
              y=0.34,
              width=0.6,
              height=0.6),
            Documentation(info="Each cylinder is shifted on that crank shaft.  This model enforces an angular displacement from one flange to another
so that each cylinder can be rigidly connected to the crankshaft by independently shifted.
"));
        end OffsetShaft;

        model CrankSlider "A crank slider mechanism"
        protected
          Modelica.SIunits.Length d;
          Modelica.SIunits.Angle phi;
          Real cp;
          Real sp;
        public
          Interfaces.EngineGeometryRequired geom annotation (Placement(
                transformation(
                origin={110,0},
                extent={{-10,-10},{10,10}},
                rotation=180)));
          Modelica.Mechanics.Rotational.Interfaces.Flange_a crank annotation (Placement(
                transformation(extent={{-10,-50},{10,-30}}, rotation=0)));
          Modelica.Mechanics.Translational.Interfaces.Flange_a piston annotation (Placement(
                transformation(extent={{-10,90},{10,110}}, rotation=0)));
        equation
          assert(geom.conrod > geom.crank,
            "Connecting rod length greater than crank length");
          cp = Modelica.Math.cos(phi);
          sp = Modelica.Math.sin(phi);
          phi = crank.phi;
          d = sqrt(geom.conrod^2 - (geom.crank*sp)^2);
          piston.s = (geom.crank + geom.conrod) - (geom.crank*cp + sqrt(geom.
            conrod^2 - geom.crank^2*sp^2));
          crank.tau = piston.f*(sp*geom.crank + cp*sp*geom.crank^2/sqrt(d));
          annotation (
            Icon(coordinateSystem(
                preserveAspectRatio=false,
                extent={{-100,-100},{100,100}},
                grid={2,2}), graphics={
                Polygon(
                  points={{-18,-40},{-46,-44},{-50,-60},{-20,-90},{-4,-86},{0,
                      -60},{-18,-40}},
                  lineColor={192,192,192},
                  fillColor={192,192,192},
                  fillPattern=FillPattern.Solid),
                Ellipse(
                  extent={{-18,118},{18,82}},
                  lineColor={160,160,164},
                  fillColor={160,160,164},
                  fillPattern=FillPattern.Solid),
                Ellipse(
                  extent={{-14,114},{14,86}},
                  lineColor={0,0,0},
                  fillColor={255,255,255},
                  fillPattern=FillPattern.Solid),
                Ellipse(
                  extent={{0,14},{54,-38}},
                  lineColor={192,192,192},
                  fillColor={160,160,164},
                  fillPattern=FillPattern.Solid),
                Ellipse(
                  extent={{12,-26},{42,4}},
                  lineColor={192,192,192},
                  fillColor={192,192,192},
                  fillPattern=FillPattern.Solid),
                Ellipse(
                  extent={{-30,-10},{30,-70}},
                  lineColor={192,192,192},
                  fillColor={192,192,192},
                  fillPattern=FillPattern.Solid),
                Polygon(
                  points={{-16,92},{2,-20},{50,2},{18,102},{12,88},{4,84},{-8,
                      86},{-16,92}},
                  lineColor={160,160,164},
                  fillColor={160,160,164},
                  fillPattern=FillPattern.Solid),
                Polygon(
                  points={{-6,-14},{20,0},{38,-18},{24,-46},{18,-26},{-6,-18},
                      {-6,-14}},
                  lineColor={192,192,192},
                  fillColor={192,192,192},
                  fillPattern=FillPattern.Solid),
                Ellipse(
                  extent={{12,-26},{42,4}},
                  lineColor={192,192,192},
                  fillColor={192,192,192},
                  fillPattern=FillPattern.Solid),
                Polygon(
                  points={{18,106},{20,68},{32,26},{58,-10},{72,70},{34,102},
                      {18,106}},
                  lineColor={0,0,255},
                  pattern=LinePattern.None,
                  fillColor={255,255,255},
                  fillPattern=FillPattern.Solid),
                Polygon(
                  points={{-20,102},{-4,64},{4,20},{0,-8},{-28,14},{-20,102}},
                  lineColor={0,0,255},
                  pattern=LinePattern.None,
                  fillColor={255,255,255},
                  fillPattern=FillPattern.Solid),
                Text(extent={{-62,-100},{66,-120}}, textString=
                                                        "%name")}),
            Window(
              x=0.54,
              y=0.23,
              width=0.29,
              height=0.64),
            Documentation(info="This model represents the crank slider mechanism used to turn translational
force into the rotational torque applied to the crankshaft.
"),         Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,
                    -100},{100,100}},
                grid={2,2}),           graphics));
        end CrankSlider;

        model SparkControl "Spark plug control"
          parameter Types.Degrees spark_advance=20
            "Before top-dead-center (TDC)";
        protected
          Types.Degrees cur_pos;
          Types.Degrees next_spark;
        public
          Modelica.Blocks.Interfaces.BooleanOutput spark  annotation (Placement(
                transformation(extent={{100,-10},{120,10}}, rotation=0)));
          Modelica.Mechanics.Rotational.Interfaces.Flange_a crank annotation (Placement(
                transformation(extent={{-10,-110},{10,-90}}, rotation=0)));
        equation
          crank.tau = 0;
          cur_pos = crank.phi*180/Modelica.Constants.pi;
        algorithm
          when initial() then
            next_spark := -spark_advance;
            while (next_spark < cur_pos) loop
              next_spark := next_spark + 720;
            end while;
          end when;
          spark:= cur_pos > next_spark;
          when spark then
            next_spark := next_spark + 720;
          end when;
          annotation (
            Diagram(coordinateSystem(
                preserveAspectRatio=false,
                extent={{-100,-100},{100,100}},
                grid={2,2}), graphics),
            Icon(coordinateSystem(
                preserveAspectRatio=false,
                extent={{-100,-100},{100,100}},
                grid={2,2}), graphics={
                Rectangle(
                  extent={{-100,100},{100,-100}},
                  lineColor={0,0,255},
                  fillColor={223,223,159},
                  fillPattern=FillPattern.Solid),
                Rectangle(
                  extent={{-20,60},{14,-20}},
                  lineColor={0,0,0},
                  fillColor={160,160,164},
                  fillPattern=FillPattern.Solid),
                Polygon(
                  points={{14,-20},{14,-46},{-6,-50},{-6,-44},{8,-42},{8,-20},
                      {14,-20}},
                  lineColor={0,0,0},
                  fillColor={160,160,164},
                  fillPattern=FillPattern.Solid),
                Polygon(
                  points={{-8,-24},{-34,-46},{-14,-36},{-20,-54},{-6,-36},{0,
                      -58},{2,-36},{16,-56},{12,-36},{40,-52},{6,-24},{4,-26},
                      {-4,-26},{-8,-24}},
                  lineColor={255,127,0},
                  fillColor={255,255,0},
                  fillPattern=FillPattern.Solid),
                Line(points={{60,0},{100,0}}, color={255,0,255}),
                Text(extent={{-64,120},{56,100}}, textString=
                                                      "Spark Controller"),
                Text(extent={{-88,-50},{96,-88}}, textString=
                    "Timing: %spark_advance [deg] before TDC")}),
            Window(
              x=0.18,
              y=0.22,
              width=0.38,
              height=0.67),
            Documentation(info="This model triggers the firing of the spark plug when the piston reaches a prescribed number of degrees
before top dead center of the compression/combustion strokes.  A real spark control strategy would allow
the spark strategy to change as engine conditions changed but this model just assumes a fixed
\"spark advance\".
"));
        end SparkControl;

        model TimingBelt "Engine timing belt"

          Modelica.Mechanics.Rotational.Interfaces.Flange_a crankshaft annotation (Placement(
                transformation(extent={{-10,-90},{10,-70}}, rotation=0)));
          Modelica.Mechanics.Rotational.Interfaces.Flange_b camshaft annotation (Placement(
                transformation(extent={{-10,50},{10,70}}, rotation=0)));
        equation
          2*camshaft.phi = crankshaft.phi;
          camshaft.tau = 2*crankshaft.tau;
          annotation (
            Icon(coordinateSystem(
                preserveAspectRatio=false,
                extent={{-100,-100},{100,100}},
                grid={2,2}), graphics={
                Ellipse(
                  extent={{-40,100},{40,20}},
                  lineColor={0,0,0},
                  fillColor={192,192,192},
                  fillPattern=FillPattern.Solid),
                Ellipse(
                  extent={{20,-100},{-20,-60}},
                  lineColor={0,0,0},
                  fillColor={192,192,192},
                  fillPattern=FillPattern.Solid),
                Line(
                  points={{-20,-82},{-40,56}},
                  color={192,192,192},
                  thickness=1),
                Line(
                  points={{-40,56},{-20,-82}},
                  color={0,0,0},
                  pattern=LinePattern.Dash),
                Line(
                  points={{40,56},{20,-82}},
                  color={192,192,192},
                  thickness=1),
                Line(
                  points={{40,54},{20,-82}},
                  color={0,0,0},
                  pattern=LinePattern.Dash),
                Text(extent={{-60,0},{60,-20}}, textString=
                                                    "%name")}),
            Window(
              x=0.3,
              y=0.17,
              width=0.6,
              height=0.6),
            Documentation(info="A timing belt that makes sure the camshaft spins at half the frequency as the crank shaft.
"));
        end TimingBelt;

        model Combustion "Simple combustion model"
          parameter Modelica.SIunits.SpecificEnergy lhv=44e+6
            "Lower heating value";
          parameter Real afr=14.6 "Stoichiometric Air/Fuel Ratio";
          parameter Types.Degrees burn_duration=60 "Duration of combustion";
        protected
          Modelica.SIunits.Energy amplitude;
          Modelica.SIunits.AngularVelocity w;
          Real dps;
          Modelica.SIunits.Time start_burn(start=-1);
          Modelica.SIunits.Time end_burn(start=-1);
          Boolean burning(start=false);
          Real tmp;
        public
          Interfaces.Gas cylinder annotation (Placement(transformation(extent=
                   {{-10,-50},{10,-30}}, rotation=0)));
          Modelica.Blocks.Interfaces.RealInput mass         annotation (Placement(
                transformation(
                origin={-60,-110},
                extent={{10,-10},{-10,10}},
                rotation=270)));
          Modelica.Blocks.Interfaces.BooleanInput start             annotation (Placement(
                transformation(
                origin={0,110},
                extent={{-10,-10},{10,10}},
                rotation=270)));
          Modelica.Mechanics.Rotational.Interfaces.Flange_a crank annotation (Placement(
                transformation(extent={{-110,-50},{-90,-30}}, rotation=0)));
        equation
          assert(burn_duration > 1, "Invalid burn duration");
          cylinder.mdot = 0;
          cylinder.q = if (burning) then -amplitude*Modelica.Math.sin((time -
            start_burn)/(end_burn - start_burn)*Modelica.Constants.pi)^2 else 0.0;
          der(tmp) = cylinder.q;
          w = der(crank.phi);
          dps = w*180/Modelica.Constants.pi;
          crank.tau = 0;
        algorithm
          when start then
            start_burn := time;
            end_burn := time + (burn_duration/dps);
            amplitude := lhv*(mass/(afr + 1))*2.0*dps/burn_duration;
            burning := true;
          end when;
          when time >= end_burn then
            burning := false;
          end when;
          annotation (
            Window(
              x=0.41,
              y=0.28,
              width=0.26,
              height=0.6),
            Icon(coordinateSystem(
                preserveAspectRatio=false,
                extent={{-100,-100},{100,100}},
                grid={2,2}), graphics={
                Rectangle(
                  extent={{-8,40},{8,10}},
                  lineColor={0,0,0},
                  fillPattern=FillPattern.VerticalCylinder,
                  fillColor={192,192,192}),
                Polygon(
                  points={{4,10},{4,2},{-2,2},{-2,0},{6,0},{6,10},{4,10}},
                  lineColor={0,0,0},
                  fillColor={192,192,192},
                  fillPattern=FillPattern.Solid),
                Rectangle(
                  extent={{-2,10},{2,6}},
                  lineColor={0,0,0},
                  fillColor={160,160,164},
                  fillPattern=FillPattern.Solid),
                Polygon(
                  points={{-12,4},{-6,-2},{8,-2},{14,4},{22,4},{16,0},{20,-6},
                      {12,-6},{10,-12},{6,-6},{4,-18},{0,-8},{-6,-14},{-6,-6},
                      {-16,-8},{-12,-2},{-20,2},{-12,4}},
                  lineColor={255,255,0},
                  fillColor={255,255,0},
                  fillPattern=FillPattern.Solid),
                Polygon(
                  points={{-20,2},{-26,2},{-16,-2},{-20,-12},{-8,-10},{-10,
                      -20},{-2,-16},{6,-26},{8,-16},{12,-18},{16,-10},{26,-8},
                      {22,-2},{30,4},{22,4},{16,0},{20,-6},{12,-6},{10,-12},{
                      6,-8},{4,-18},{0,-8},{-6,-14},{-6,-6},{-14,-8},{-12,-2},
                      {-20,2}},
                  lineColor={255,0,0},
                  fillColor={255,127,0},
                  fillPattern=FillPattern.Solid),
                Polygon(
                  points={{-80,-80},{-80,-20},{-28,0},{-28,10},{-8,10},{-8,40},
                      {-92,40},{-92,-80},{-80,-80}},
                  lineColor={0,0,0},
                  fillColor={255,255,255},
                  fillPattern=FillPattern.Forward),
                Polygon(
                  points={{8,40},{8,10},{32,10},{32,0},{80,-20},{80,-80},{90,
                      -80},{90,40},{8,40},{8,10},{8,40}},
                  lineColor={0,0,0},
                  fillColor={255,255,255},
                  fillPattern=FillPattern.Forward),
                Line(points={{0,112},{0,40}}, color={255,0,255}),
                Text(extent={{-60,-80},{60,-100}}, textString=
                                                       "%name")}),
            Documentation(info="This is a simplified combustion models.  This
model works by computing an instantaneous heat
release (i.e. energy given off as a result
of combustion) based on an idealized combustion
transient.  The duration of the burning is
typically determined by the motion of the
fuel/air mixture inside the cylinder but in
the case of this model it is assumed that
the burn duration is a fixed parameter.
"),         Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,
                    -100},{100,100}},
                grid={2,2}),           graphics));
        end Combustion;

        model Dynamometer "Ideal Dynamometer"
          parameter Real cycle_fraction=1.0
            "Average over what fraction of the cycle";
          Modelica.SIunits.Energy work;
          Modelica.SIunits.Energy last_work;
          Modelica.SIunits.Torque avg_tau;
          Modelica.SIunits.Angle next_rotation;
          Types.RPM avg_rpm;
          Modelica.SIunits.Time previous_time;
          Modelica.SIunits.Power Power;

          Modelica.Blocks.Interfaces.RealInput rpm         annotation (Placement(
                transformation(extent={{-120,-10},{-100,10}}, rotation=0)));
          Modelica.Mechanics.Rotational.Interfaces.Flange_a shaft annotation (Placement(
                transformation(extent={{90,-10},{110,10}}, rotation=0)));
        equation
          der(shaft.phi) = rpm*Modelica.Constants.pi/30;
          der(work) = shaft.tau*der(shaft.phi);
          Power=rpm*Modelica.Constants.pi/30*avg_tau;

        algorithm
          when initial() then
            next_rotation := shaft.phi + cycle_fraction*4*Modelica.Constants.pi;
            last_work := 0;
            previous_time := time;
            avg_rpm := rpm;
          end when;
          when shaft.phi > next_rotation then
            next_rotation := next_rotation + cycle_fraction*4*Modelica.Constants.
              pi;
            avg_tau := (work - pre(last_work))/(cycle_fraction*4*Modelica.
              Constants.pi);
            avg_rpm := 120*cycle_fraction/(time - pre(previous_time));
            last_work := work;
            previous_time := time;
          end when;
          annotation (
            Icon(coordinateSystem(
                preserveAspectRatio=false,
                extent={{-100,-100},{100,100}},
                grid={2,2}), graphics={
                Rectangle(
                  extent={{60,10},{90,-10}},
                  lineColor={0,0,0},
                  fillPattern=FillPattern.HorizontalCylinder,
                  fillColor={192,192,192}),
                Rectangle(
                  extent={{-40,60},{40,-60}},
                  lineColor={0,0,0},
                  pattern=LinePattern.Solid,
                  lineThickness=0.25,
                  fillPattern=FillPattern.HorizontalCylinder,
                  fillColor={192,192,192}),
                Polygon(
                  points={{-60,10},{-60,20},{-40,40},{-40,-40},{-60,-20},{-60,
                      10}},
                  lineColor={0,0,0},
                  fillPattern=FillPattern.HorizontalCylinder,
                  fillColor={128,128,128}),
                Polygon(
                  points={{60,20},{40,40},{40,-40},{60,-20},{60,20}},
                  lineColor={128,128,128},
                  fillColor={128,128,128},
                  fillPattern=FillPattern.Solid),
                Polygon(
                  points={{-60,-90},{-50,-90},{-20,-30},{20,-30},{48,-90},{60,
                      -90},{60,-100},{-60,-100},{-60,-90}},
                  lineColor={0,0,0},
                  fillColor={0,0,0},
                  fillPattern=FillPattern.Solid),
                Line(points={{-112,0},{-60,0}}),
                Text(extent={{-80,100},{80,60}}, textString=
                                                     "Dynamometer")}),
            Window(
              x=0.38,
              y=0.22,
              width=0.6,
              height=0.6),
            Documentation(info="This dynamometer model is ideal.  This means that the dynamometer appear (to anything connected to it) to have
an infinite mass.  It is important that the input signal is continuous.  Furthermore the input signal has units
of \"revolutions per minute\".
"));
        end Dynamometer;

        model IndividualCylinder
          "Collection of parts for a complete individual cylinder"
          extends Interfaces.Cylinder;
          parameter Types.Degrees spark_advance "Spark advance";
          parameter Types.Degrees burn_duration "Burn Duration";
          parameter Types.Degrees evo=40 "Exhaust Valve Opening";
          parameter Types.Degrees ivo=150 "Intake Valve Opening";
          parameter Types.Degrees evc=205 "Exhaust Valve Closing";
          parameter Types.Degrees ivc=310 "Intake Valve Closing";
          parameter Types.Degrees crank_shift=0 "Crankshaft Shift";
          parameter Modelica.SIunits.Diameter ivd=0.032 "Intake Valve Diameter";
          parameter Modelica.SIunits.Diameter evd=0.028
            "Exhaust Valve Diameter";
          Engine.Components.MasslessPiston piston annotation (
                        Placement(transformation(extent={{-10,-50},{10,-30}},
                  rotation=0)));
          Engine.Components.CrankSlider crankslider annotation (
                            Placement(transformation(extent={{-20,-120},{20,
                    -80}}, rotation=0)));
          Engine.Components.ControlVolume combustion_chamber annotation (Placement(
                transformation(extent={{-10,-20},{10,0}}, rotation=0)));
          Engine.Components.Valve intake_valve(dia=ivd) annotation (
                              Placement(transformation(extent={{-40,20},{-20,
                    40}}, rotation=0)));
          Engine.Components.Valve exhaust_valve(dia=evd) annotation (
                             Placement(transformation(extent={{40,20},{20,40}},
                  rotation=0)));
          Engine.Components.TimingBelt timing_belt annotation (Placement(
                transformation(extent={{-100,-60},{-60,-20}}, rotation=0)));
          Engine.Components.Cam intake_cam(vo=ivo, vc=ivc) annotation (
                                Placement(transformation(extent={{-60,60},{
                    -40,80}}, rotation=0)));
          Engine.Components.Cam exhaust_cam(vo=evo, vc=evc) annotation (
                               Placement(transformation(extent={{60,60},{40,
                    80}}, rotation=0)));
          Engine.Components.Combustion combustion_model(burn_duration=
                burn_duration) annotation (                          Placement(
                transformation(extent={{-10,20},{10,40}}, rotation=0)));
          Engine.Components.SparkControl spark_control(spark_advance=spark_advance) 
              annotation (                          Placement(transformation(
                  extent={{-24,44},{-4,64}}, rotation=0)));
          Engine.Components.ChamberVolume chamber_volume annotation (
                            Placement(transformation(extent={{56,-20},{76,0}},
                  rotation=0)));
          OffsetShaft offset_shaft(shift=crank_shift) annotation (
                                Placement(transformation(extent={{-20,-140},{
                    20,-180}}, rotation=0)));
        equation
          connect(crankslider.piston, chamber_volume.piston) annotation (Line(
              points={{0,-80},{66,-80},{66,-18}},
              color={0,127,0},
              smooth=Smooth.None));
          connect(crankslider.crank, offset_shaft.cyl) annotation (Line(
              points={{0,-108},{0,-140}},
              color={0,0,0},
              smooth=Smooth.None));
          connect(crankslider.crank, timing_belt.crankshaft) annotation (Line(
              points={{0,-108},{-80,-108},{-80,-56}},
              color={0,0,0},
              smooth=Smooth.None));
          connect(crankslider.crank, spark_control.crank) annotation (Line(
              points={{0,-108},{-14,-108},{-14,44}},
              color={0,0,0},
              smooth=Smooth.None));
          connect(offset_shaft.crank, crankshaft) annotation (Line(
              points={{-20,-160},{-20,-198},{0,-198}},
              color={0,0,0},
              smooth=Smooth.None));
          connect(timing_belt.camshaft, exhaust_cam.camshaft) annotation (Line(
              points={{-80,-28},{-80,86},{60,86},{60,70},{52.8,70}},
              color={0,0,0},
              smooth=Smooth.None));
          connect(timing_belt.camshaft, intake_cam.camshaft) annotation (Line(
              points={{-80,-28},{-80,70},{-52.8,70}},
              color={0,0,0},
              smooth=Smooth.None));
          connect(intake_valve.a, intake) annotation (Line(
              points={{-40,30},{-100,30},{-100,80}},
              color={255,0,0},
              smooth=Smooth.None));
          connect(exhaust_valve.a, exhaust) annotation (Line(
              points={{40,30},{100,30},{100,80}},
              color={255,0,0},
              smooth=Smooth.None));
          connect(intake_cam.valve_lift, intake_valve.lift) annotation (Line(
              points={{-40,70},{-40,60.5},{-32,60.5},{-32,40},{-30,40}},
              color={0,127,0},
              smooth=Smooth.None));
          connect(exhaust_cam.valve_lift, exhaust_valve.lift) annotation (Line(
              points={{40,70},{40,62.5},{24,62.5},{24,55},{30,55},{30,40}},
              color={0,127,0},
              smooth=Smooth.None));
          connect(spark_control.spark, combustion_model.start) annotation (Line(
              points={{-3,54},{0,54},{0,41}},
              color={255,0,255},
              smooth=Smooth.None));
          connect(crankslider.crank, combustion_model.crank) annotation (Line(
              points={{0,-108},{-54,-108},{-54,26},{-10,26}},
              color={0,0,0},
              smooth=Smooth.None));
          connect(combustion_model.mass, combustion_chamber.mass) annotation (
              Line(
              points={{-6,19},{-6,1}},
              color={0,0,127},
              smooth=Smooth.None));
          connect(combustion_model.cylinder, combustion_chamber.state) 
            annotation (Line(
              points={{0,26},{0,-10}},
              color={255,0,0},
              smooth=Smooth.None));
          connect(combustion_chamber.state, piston.chamber) annotation (Line(
              points={{0,-10},{0,-30}},
              color={255,0,0},
              smooth=Smooth.None));
          connect(piston.piston, crankslider.piston) annotation (Line(
              points={{0,-42.2},{0,-80}},
              color={0,127,0},
              smooth=Smooth.None));
          connect(chamber_volume.volume, combustion_chamber.volume) annotation (
             Line(
              points={{55,-10},{11,-10}},
              color={0,0,127},
              smooth=Smooth.None));
          connect(combustion_chamber.state, exhaust_valve.b) annotation (Line(
              points={{0,-10},{6,-10},{6,12},{30,12},{30,20}},
              color={255,0,0},
              smooth=Smooth.None));
          connect(combustion_chamber.state, intake_valve.b) annotation (Line(
              points={{0,-10},{-12,-10},{-12,12},{-30,12},{-30,20}},
              color={255,0,0},
              smooth=Smooth.None));
          connect(geom, chamber_volume.geom) annotation (Line(
              points={{110,-50},{80,-50},{80,-10},{77,-10}},
              color={0,0,0},
              smooth=Smooth.None));
          connect(geom, piston.geom) annotation (Line(
              points={{110,-50},{70,-50},{70,-40},{11,-40}},
              color={0,0,0},
              smooth=Smooth.None));
          connect(geom, crankslider.geom) annotation (Line(
              points={{110,-50},{80,-50},{80,-100},{22,-100}},
              color={0,0,0},
              smooth=Smooth.None));
          annotation (
            Window(
              x=0.07,
              y=0.03,
              width=0.99,
              height=0.91),
            Icon(coordinateSystem(
                preserveAspectRatio=false,
                extent={{-100,-200},{100,100}},
                grid={2,2}), graphics={
                Polygon(
                  points={{-60,-140},{-60,40},{-42,40},{-42,44},{-40,46},{-60,
                      60},{-100,60},{-100,-140},{-60,-140}},
                  lineColor={0,0,0},
                  fillColor={192,192,192},
                  fillPattern=FillPattern.Solid),
                Polygon(
                  points={{-100,96},{-60,96},{-40,80},{-20,60},{-20,46},{-18,
                      44},{-18,40},{18,40},{18,44},{20,46},{20,60},{40,80},{
                      60,96},{100,96},{100,100},{-100,100},{-100,96}},
                  lineColor={0,0,0},
                  fillColor={192,192,192},
                  fillPattern=FillPattern.Solid),
                Rectangle(
                  extent={{-58,12},{58,-64}},
                  lineColor={0,0,0},
                  fillPattern=FillPattern.VerticalCylinder,
                  fillColor={192,192,192}),
                Rectangle(
                  extent={{-60,6},{60,0}},
                  lineColor={0,0,0},
                  fillColor={0,0,0},
                  fillPattern=FillPattern.Solid),
                Rectangle(
                  extent={{-60,-8},{60,-14}},
                  lineColor={0,0,0},
                  fillColor={0,0,0},
                  fillPattern=FillPattern.Solid),
                Rectangle(
                  extent={{-60,-20},{60,-26}},
                  lineColor={0,0,0},
                  fillColor={0,0,0},
                  fillPattern=FillPattern.Solid),
                Polygon(
                  points={{-58,-64},{-40,-52},{40,-52},{58,-64},{-58,-64}},
                  lineColor={0,0,255},
                  pattern=LinePattern.None,
                  fillColor={255,255,255},
                  fillPattern=FillPattern.Solid),
                Ellipse(
                  extent={{-4,-34},{4,-42}},
                  lineColor={0,0,0},
                  fillColor={0,0,0},
                  fillPattern=FillPattern.Solid),
                Ellipse(extent={{-40,-240},{40,-160}}, lineColor={192,192,192}),
                Line(
                  points={{0,-200},{30,-174},{0,-38}},
                  color={0,0,0},
                  thickness=1),
                Rectangle(
                  extent={{-42,36},{-18,32}},
                  lineColor={0,0,0},
                  fillPattern=FillPattern.VerticalCylinder,
                  fillColor={192,192,192}),
                Polygon(
                  points={{-42,36},{-40,38},{-20,38},{-18,36},{-42,36}},
                  lineColor={160,160,164},
                  fillColor={192,192,192},
                  fillPattern=FillPattern.Solid),
                Rectangle(
                  extent={{-32,38},{-28,96}},
                  lineColor={0,0,0},
                  fillPattern=FillPattern.VerticalCylinder,
                  fillColor={192,192,192}),
                Polygon(
                  points={{18,44},{20,46},{40,46},{42,44},{18,44}},
                  lineColor={160,160,164},
                  fillColor={192,192,192},
                  fillPattern=FillPattern.Solid),
                Rectangle(
                  extent={{18,44},{42,40}},
                  lineColor={0,0,0},
                  fillPattern=FillPattern.VerticalCylinder,
                  fillColor={192,192,192}),
                Rectangle(
                  extent={{28,46},{32,96}},
                  lineColor={0,0,0},
                  fillPattern=FillPattern.VerticalCylinder,
                  fillColor={192,192,192}),
                Polygon(
                  points={{100,-140},{100,60},{60,60},{40,46},{42,44},{42,40},
                      {60,40},{60,-140},{100,-140}},
                  lineColor={0,0,0},
                  fillColor={192,192,192},
                  fillPattern=FillPattern.Solid),
                Text(extent={{-60,-100},{60,-120}}, textString=
                                                        "%name")}),
            Documentation(info="All the basic componets of a single cylinder, 2-valve engine
have been compiled in this model.
"),         Diagram(coordinateSystem(preserveAspectRatio=true, extent={{-100,
                    -200},{100,100}},
                grid={2,2}),           graphics));
        end IndividualCylinder;

        model Cam "Valvetrain cam"

          parameter Types.Degrees vo=40 "Valve Open";
          parameter Types.Degrees vc=205 "Valve Close";
          parameter Modelica.SIunits.Length max_lift=0.012 "Maximum valve lift";
          Types.Degrees local_ca "Local camshaft angle";
          Real deg_T "Time in degrees";
        protected
          parameter Real norm=1.0/(vc - vo);
        public
          Modelica.Mechanics.Rotational.Interfaces.Flange_a camshaft annotation (Placement(
                transformation(extent={{-38,-10},{-18,10}}, rotation=0)));
          Modelica.Mechanics.Translational.Interfaces.Flange_a valve_lift 
            annotation (Placement(transformation(extent={{90,-10},{110,10}},
                  rotation=0)));
        equation
          // assert(vc > vo + 20, "Invalid cam timings");
          camshaft.tau = 0;
          valve_lift.s = if (local_ca < vo or local_ca > vc) then 0.0 else 
            max_lift*Modelica.Math.sin((local_ca - vo)*norm*Modelica.Constants.pi)^2;
          local_ca = mod(camshaft.phi*180.0/Modelica.Constants.pi, 360);
          deg_T=camshaft.phi*360.0/Modelica.Constants.pi;

          annotation (
            Icon(coordinateSystem(
                preserveAspectRatio=false,
                extent={{-100,-100},{100,100}},
                grid={2,2}), graphics={
                Ellipse(
                  extent={{-78,50},{22,-50}},
                  lineColor={192,192,192},
                  fillColor={192,192,192},
                  fillPattern=FillPattern.Solid),
                Polygon(
                  points={{-4,44},{30,26},{40,14},{44,0},{38,-20},{24,-32},{
                      10,-38},{-8,-46},{-4,44}},
                  lineColor={192,192,192},
                  fillColor={192,192,192},
                  fillPattern=FillPattern.Solid),
                Rectangle(
                  extent={{54,4},{90,-4}},
                  lineColor={0,0,0},
                  fillPattern=FillPattern.HorizontalCylinder,
                  fillColor={192,192,192}),
                Ellipse(
                  extent={{44,6},{56,-6}},
                  lineColor={0,0,0},
                  fillColor={160,160,164},
                  fillPattern=FillPattern.Solid),
                Text(extent={{-40,80},{40,60}}, textString=
                                                    "%name")}),
            Window(
              x=0.19,
              y=0.2,
              width=0.6,
              height=0.6),
            Documentation(info="This is an idealized cam model that computes a cam profile from the valve timing (opening and closing) and the
maximum lift.
"));
        end Cam;

        model Reservoir "Infinite reservoir"
          parameter Modelica.SIunits.Pressure P=101800 "Reservoir pressure";
          parameter Modelica.SIunits.Temperature T=300 "Reservoir temperature";
          Interfaces.Gas tap annotation (                             layer="icon",
              Placement(transformation(extent={{-10,-110},{10,-90}}, rotation=
                   0)));
        equation
          tap.P = P;
          tap.T = T;
          annotation (
            Diagram(coordinateSystem(
                preserveAspectRatio=false,
                extent={{-100,-100},{100,100}},
                grid={2,2}), graphics),
            Window(
              x=0.39,
              y=0.26,
              width=0.6,
              height=0.6),
            Icon(coordinateSystem(
                preserveAspectRatio=false,
                extent={{-100,-100},{100,100}},
                grid={2,2}), graphics={
                Rectangle(
                  extent={{-100,100},{100,-90}},
                  lineColor={0,0,0},
                  fillColor={255,255,255},
                  fillPattern=FillPattern.Solid),
                Text(extent={{-42,90},{34,66}}, textString=
                                                    "P=%P"),
                Line(points={{-60,10},{-40,30},{-20,30},{0,10},{0,-10},{20,
                      -30},{40,-30},{60,-10},{60,10},{40,30},{20,30},{0,10},{
                      0,-10},{-20,-30},{-40,-30},{-60,-10},{-60,10}}, color={
                      0,0,0}),
                Text(extent={{-40,70},{36,46}}, textString=
                                                    "T=%T"),
                Text(extent={{-60,-74},{60,-52}}, textString=
                                                      "%name")}),
            Documentation(info="This infinite reservoir model is capable of supplying any amount of mass or energy (via the mdot and q flow variables on the
Gas connector) to sustain a specified temperature and pressure.

These \"infinite\" models are interesting because instead of including constitutive relationships between
flow values (mass flow and power) and the potentials of the system (pressure and temperature), they instead
provide algebraic constraints involving only the states.  It is assumed that the flows will be calculated
such that the constraints will be satisfied.  This kind of model is analagous to an ideal voltage source (supplies
as much current as is necessary to sustain a specified voltage) or a mechanical ground (supplies as much
force as is necessary to sustain a specified position).
"));
        end Reservoir;

        model I4_Engine "An Inline 4 Cylinder Engine"
          extends Interfaces.Engine;
          parameter Types.Degrees spark_advance "Spark advance";
          parameter Types.Degrees burn_duration "Burn Duration";
          parameter Types.Degrees evo=40 "Exhaust Valve Opening";
          parameter Types.Degrees ivo=150 "Intake Valve Opening";
          parameter Types.Degrees evc=205 "Exhaust Valve Closing";
          parameter Types.Degrees ivc=310 "Intake Valve Closing";
          parameter Modelica.SIunits.Diameter ivd=0.032 "Intake Valve Diameter";
          parameter Modelica.SIunits.Diameter evd=0.028
            "Exhaust Valve Diameter";
          Modelica.SIunits.Power power "Instantaneous engine power output";
          replaceable model CylinderType = IndividualCylinder constrainedby
            Interfaces.Cylinder;
          Modelica.Mechanics.Rotational.Components.Inertia crankshaft_inertia(
                                                                   J=0.03, w(start=
                 157)) annotation (Placement(transformation(
                origin={-82,-50},
                extent={{-10,-10},{10,10}},
                rotation=90)));
          IndividualCylinder_Weibe_Hohenberg_Biogas
            individualCylinder_Weibe_Hohenberg_Biogas(
            spark_advance=spark_advance,
            burn_duration=burn_duration,
            evo=evo,
            ivo=ivo,
            evc=evc,
            ivc=ivc,
            crank_shift=0,
            ivd=ivd,
            evd=evd) annotation (Placement(transformation(extent={{-100,-20},{
                    -60,40}})));
          IndividualCylinder_Weibe_Hohenberg_Biogas
            individualCylinder_Weibe_Hohenberg_Biogas1(
            spark_advance=spark_advance,
            burn_duration=burn_duration,
            evo=evo,
            ivo=ivo,
            evc=evc,
            ivc=ivc,
            crank_shift=0,
            ivd=ivd,
            evd=evd) 
            annotation (Placement(transformation(extent={{-52,-20},{-12,40}})));
          IndividualCylinder_Weibe_Hohenberg_Biogas
            individualCylinder_Weibe_Hohenberg_Biogas2(
            spark_advance=spark_advance,
            burn_duration=burn_duration,
            evo=evo,
            ivo=ivo,
            evc=evc,
            ivc=ivc,
            crank_shift=0,
            ivd=ivd,
            evd=evd) 
            annotation (Placement(transformation(extent={{0,-20},{40,40}})));
          IndividualCylinder_Weibe_Hohenberg_Biogas
            individualCylinder_Weibe_Hohenberg_Biogas3(
            spark_advance=spark_advance,
            burn_duration=burn_duration,
            evo=evo,
            ivo=ivo,
            evc=evc,
            ivc=ivc,
            crank_shift=0,
            ivd=ivd,
            evd=evd) 
            annotation (Placement(transformation(extent={{48,-20},{88,40}})));
          Interfaces.BiogasPropertyRequired biogasPropertyRequired 
            annotation (Placement(transformation(extent={{98,-56},{118,-36}})));
        equation
          connect(crankshaft_inertia.flange_a, crankshaft) annotation (Line(
                points={{-82,-60},{-82,-70},{-100,-70},{-100,-40}}, color={0,
                  0,0}));
          power = crankshaft.tau*der(crankshaft.phi);
          connect(individualCylinder_Weibe_Hohenberg_Biogas.crankshaft,
            individualCylinder_Weibe_Hohenberg_Biogas1.crankshaft) annotation (
              Line(
              points={{-80,-19.6},{-32,-19.6}},
              color={0,0,0},
              smooth=Smooth.None));
          connect(individualCylinder_Weibe_Hohenberg_Biogas1.crankshaft,
            individualCylinder_Weibe_Hohenberg_Biogas2.crankshaft) annotation (
              Line(
              points={{-32,-19.6},{20,-19.6}},
              color={0,0,0},
              smooth=Smooth.None));
          connect(individualCylinder_Weibe_Hohenberg_Biogas2.crankshaft,
            individualCylinder_Weibe_Hohenberg_Biogas3.crankshaft) annotation (
              Line(
              points={{20,-19.6},{68,-19.6}},
              color={0,0,0},
              smooth=Smooth.None));
          connect(crankshaft_inertia.flange_b,
            individualCylinder_Weibe_Hohenberg_Biogas.crankshaft) annotation (
              Line(
              points={{-82,-40},{-82,-30},{-80,-30},{-80,-19.6}},
              color={0,0,0},
              smooth=Smooth.None));
          connect(biogasPropertyRequired,
            individualCylinder_Weibe_Hohenberg_Biogas3.bioP) annotation (Line(
              points={{108,-46},{92,-46},{92,21.6},{90,21.6}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(biogasPropertyRequired,
            individualCylinder_Weibe_Hohenberg_Biogas1.bioP) annotation (Line(
              points={{108,-46},{-4,-46},{-4,21.6},{-10,21.6}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(individualCylinder_Weibe_Hohenberg_Biogas2.bioP,
            biogasPropertyRequired) annotation (Line(
              points={{42,21.6},{46,21.6},{46,-46},{108,-46}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(individualCylinder_Weibe_Hohenberg_Biogas.bioP,
            biogasPropertyRequired) annotation (Line(
              points={{-58,21.6},{-56,21.6},{-56,22},{-54,22},{-54,-46},{108,
                  -46}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(individualCylinder_Weibe_Hohenberg_Biogas3.geom,
            engine_geometry) annotation (Line(
              points={{90,10},{96,10},{96,0},{110,0}},
              color={0,0,0},
              smooth=Smooth.None));
          connect(individualCylinder_Weibe_Hohenberg_Biogas2.geom,
            engine_geometry) annotation (Line(
              points={{42,10},{50,10},{50,-32},{96,-32},{96,0},{110,0}},
              color={0,0,0},
              smooth=Smooth.None));
          connect(individualCylinder_Weibe_Hohenberg_Biogas1.geom,
            engine_geometry) annotation (Line(
              points={{-10,10},{-6,10},{-6,-32},{96,-32},{96,0},{110,0}},
              color={0,0,0},
              smooth=Smooth.None));
          connect(individualCylinder_Weibe_Hohenberg_Biogas.geom,
            engine_geometry) annotation (Line(
              points={{-58,10},{-50,10},{-50,-32},{96,-32},{96,0},{110,0}},
              color={0,0,0},
              smooth=Smooth.None));
          connect(individualCylinder_Weibe_Hohenberg_Biogas.intake, intake) 
            annotation (Line(
              points={{-100,36},{-100,80}},
              color={255,0,0},
              smooth=Smooth.None));
          connect(individualCylinder_Weibe_Hohenberg_Biogas1.intake, intake) 
            annotation (Line(
              points={{-52,36},{-52,60},{-100,60},{-100,80}},
              color={255,0,0},
              smooth=Smooth.None));
          connect(individualCylinder_Weibe_Hohenberg_Biogas2.intake, intake) 
            annotation (Line(
              points={{0,36},{0,60},{-100,60},{-100,80}},
              color={255,0,0},
              smooth=Smooth.None));
          connect(individualCylinder_Weibe_Hohenberg_Biogas3.intake, intake) 
            annotation (Line(
              points={{48,36},{48,60},{-100,60},{-100,80}},
              color={255,0,0},
              smooth=Smooth.None));
          connect(individualCylinder_Weibe_Hohenberg_Biogas3.exhaust, exhaust) 
            annotation (Line(
              points={{88,36},{100,36},{100,80}},
              color={255,0,0},
              smooth=Smooth.None));
          connect(individualCylinder_Weibe_Hohenberg_Biogas2.exhaust, exhaust) 
            annotation (Line(
              points={{40,36},{40,52},{100,52},{100,80}},
              color={255,0,0},
              smooth=Smooth.None));
          connect(individualCylinder_Weibe_Hohenberg_Biogas1.exhaust, exhaust) 
            annotation (Line(
              points={{-12,36},{-12,52},{100,52},{100,80}},
              color={255,0,0},
              smooth=Smooth.None));
          connect(individualCylinder_Weibe_Hohenberg_Biogas.exhaust, exhaust) 
            annotation (Line(
              points={{-60,36},{-60,52},{100,52},{100,80}},
              color={255,0,0},
              smooth=Smooth.None));
          annotation (
            Icon(coordinateSystem(
                preserveAspectRatio=false,
                extent={{-100,-100},{100,100}},
                grid={2,2}), graphics={
                Polygon(
                  points={{-48,32},{-56,-8},{-56,-48},{-46,-76},{-12,-76},{0,
                      -48},{0,-8},{-8,32},{-46,32},{-48,32}},
                  lineColor={0,0,0},
                  fillColor={160,160,164},
                  fillPattern=FillPattern.Solid),
                Ellipse(
                  extent={{-46,-10},{-16,-68}},
                  lineColor={0,0,0},
                  fillColor={192,192,192},
                  fillPattern=FillPattern.Solid),
                Ellipse(
                  extent={{-36,-30},{-26,-50}},
                  lineColor={0,0,0},
                  fillPattern=FillPattern.HorizontalCylinder,
                  fillColor={192,192,192}),
                Rectangle(
                  extent={{-90,-30},{-30,-50}},
                  lineColor={0,0,0},
                  fillPattern=FillPattern.HorizontalCylinder,
                  fillColor={192,192,192}),
                Polygon(
                  points={{-8,32},{72,32},{84,-6},{84,-48},{74,-76},{-12,-76},
                      {0,-48},{0,-8},{-8,32},{-8,32}},
                  lineColor={0,0,0},
                  fillColor={192,192,192},
                  fillPattern=FillPattern.Solid),
                Line(points={{0,-6},{84,-6}}, color={160,160,164}),
                Line(points={{0,-48},{84,-48}}, color={0,0,0}),
                Rectangle(
                  extent={{-24,48},{-34,32}},
                  lineColor={0,0,0},
                  fillColor={128,128,128},
                  fillPattern=FillPattern.Solid),
                Rectangle(
                  extent={{72,48},{62,32}},
                  lineColor={0,0,0},
                  fillColor={128,128,128},
                  fillPattern=FillPattern.Solid),
                Ellipse(
                  extent={{60,70},{74,40}},
                  lineColor={0,0,0},
                  fillColor={192,192,192},
                  fillPattern=FillPattern.Solid),
                Line(points={{-36,58},{-46,-30}}, color={0,0,0}),
                Rectangle(
                  extent={{-28,62},{68,48}},
                  lineColor={0,0,0},
                  fillPattern=FillPattern.HorizontalCylinder,
                  fillColor={192,192,192}),
                Ellipse(
                  extent={{-36,70},{-24,44}},
                  lineColor={0,0,0},
                  fillColor={192,192,192},
                  fillPattern=FillPattern.Solid),
                Line(points={{-24,58},{-16,-36}}, color={0,0,0}),
                Rectangle(
                  extent={{-16,64},{-10,42}},
                  lineColor={160,160,164},
                  fillColor={160,160,164},
                  fillPattern=FillPattern.Solid),
                Rectangle(
                  extent={{-8,68},{-2,46}},
                  lineColor={160,160,164},
                  fillColor={160,160,164},
                  fillPattern=FillPattern.Solid),
                Rectangle(
                  extent={{4,66},{10,44}},
                  lineColor={160,160,164},
                  fillColor={160,160,164},
                  fillPattern=FillPattern.Solid),
                Rectangle(
                  extent={{12,64},{18,42}},
                  lineColor={160,160,164},
                  fillColor={160,160,164},
                  fillPattern=FillPattern.Solid),
                Rectangle(
                  extent={{26,68},{32,46}},
                  lineColor={160,160,164},
                  fillColor={160,160,164},
                  fillPattern=FillPattern.Solid),
                Rectangle(
                  extent={{34,66},{40,44}},
                  lineColor={160,160,164},
                  fillColor={160,160,164},
                  fillPattern=FillPattern.Solid),
                Rectangle(
                  extent={{46,64},{52,42}},
                  lineColor={160,160,164},
                  fillColor={160,160,164},
                  fillPattern=FillPattern.Solid),
                Rectangle(
                  extent={{54,66},{60,44}},
                  lineColor={160,160,164},
                  fillColor={160,160,164},
                  fillPattern=FillPattern.Solid),
                Rectangle(
                  extent={{-16,42},{-10,32}},
                  lineColor={0,0,0},
                  fillColor={192,192,192},
                  fillPattern=FillPattern.Forward),
                Rectangle(
                  extent={{-8,44},{-2,32}},
                  lineColor={0,0,0},
                  fillColor={192,192,192},
                  fillPattern=FillPattern.Forward),
                Rectangle(
                  extent={{4,44},{10,32}},
                  lineColor={0,0,0},
                  fillColor={192,192,192},
                  fillPattern=FillPattern.Forward),
                Rectangle(
                  extent={{12,42},{18,32}},
                  lineColor={0,0,0},
                  fillColor={192,192,192},
                  fillPattern=FillPattern.Forward),
                Rectangle(
                  extent={{26,44},{32,32}},
                  lineColor={0,0,0},
                  fillColor={192,192,192},
                  fillPattern=FillPattern.Forward),
                Rectangle(
                  extent={{34,44},{40,32}},
                  lineColor={0,0,0},
                  fillColor={192,192,192},
                  fillPattern=FillPattern.Forward),
                Rectangle(
                  extent={{46,42},{52,32}},
                  lineColor={0,0,0},
                  fillColor={192,192,192},
                  fillPattern=FillPattern.Forward),
                Rectangle(
                  extent={{54,44},{60,32}},
                  lineColor={0,0,0},
                  fillColor={192,192,192},
                  fillPattern=FillPattern.Forward),
                Line(
                  points={{-100,80},{-48,32}},
                  color={255,127,0},
                  thickness=0.5),
                Line(points={{100,80},{62,32}}, color={255,127,0}),
                Text(extent={{-60,-74},{80,-106}}, textString=
                                                       "%name")}),
            Window(
              x=0.39,
              y=0.34,
              width=0.6,
              height=0.6),
            Documentation(info="An assembly of components required for an I4 engine.  This model requires a connection to intake and exhaust
gases, the crankshaft and geometry information.
"),         Diagram(coordinateSystem(preserveAspectRatio=true, extent={{-100,
                    -100},{100,100}},
                grid={2,2}),           graphics));
        end I4_Engine;

        model Manifold "A simple filling-and-emptying manifold model"
          parameter Modelica.SIunits.Volume volume=0.004 "Manifold volume";
          Engine.Components.ControlVolume manifold_volume annotation (
                                Placement(transformation(extent={{-40,-40},{
                    40,40}}, rotation=0)));
          Modelica.Blocks.Sources.Constant volume_value(final k=volume) 
            annotation (                           Placement(transformation(
                  extent={{40,-60},{60,-40}}, rotation=0)));
          Interfaces.Gas ambient annotation (Placement(transformation(extent=
                    {{-10,90},{10,110}}, rotation=0)));
          Interfaces.Gas manifold annotation (                             layer=
                "icon", Placement(transformation(extent={{-10,-110},{10,-90}},
                  rotation=0)));
          Modelica.Blocks.Interfaces.RealInput throttle_angle         annotation (
                                            layer="icon", Placement(
                transformation(extent={{-120,-10},{-100,10}}, rotation=0)));
          Engine.Components.Throttle throttle(dia=0.10) annotation (Placement(
                transformation(
                origin={0,60},
                extent={{20,-20},{-20,20}},
                rotation=90)));
        equation
          connect(throttle_angle, throttle.throttle_angle) annotation (Line(
              points={{-110,0},{-56,0},{-56,60},{-22,60}},
              color={0,0,127},
              smooth=Smooth.None));
          connect(manifold_volume.state, manifold) annotation (Line(
              points={{0,0},{0,-100}},
              color={255,0,0},
              smooth=Smooth.None));
          connect(manifold_volume.state, throttle.b) annotation (Line(
              points={{0,0},{20,0},{20,60}},
              color={255,0,0},
              smooth=Smooth.None));
          connect(manifold_volume.volume, volume_value.y) annotation (Line(
              points={{44,0},{80,0},{80,-50},{61,-50}},
              color={0,0,127},
              smooth=Smooth.None));
          connect(throttle.a, ambient) annotation (Line(
              points={{1.22465e-015,80},{0,80},{0,100}},
              color={255,0,0},
              smooth=Smooth.None));
          connect(ambient, ambient) annotation (Line(
              points={{0,100},{0,97},{0,97},{0,100}},
              color={255,0,0},
              smooth=Smooth.None));
          annotation (
            Icon(coordinateSystem(
                preserveAspectRatio=false,
                extent={{-100,-100},{100,100}},
                grid={2,2}), graphics={
                Rectangle(
                  extent={{-80,40},{80,-40}},
                  lineColor={0,0,0},
                  fillPattern=FillPattern.HorizontalCylinder,
                  fillColor={192,192,192}),
                Rectangle(
                  extent={{-80,-40},{-60,-80}},
                  lineColor={0,0,0},
                  fillPattern=FillPattern.VerticalCylinder,
                  fillColor={192,192,192}),
                Rectangle(
                  extent={{20,-40},{40,-80}},
                  lineColor={0,0,0},
                  fillPattern=FillPattern.VerticalCylinder,
                  fillColor={192,192,192}),
                Rectangle(
                  extent={{-40,-40},{-20,-80}},
                  lineColor={0,0,0},
                  fillPattern=FillPattern.VerticalCylinder,
                  fillColor={192,192,192}),
                Rectangle(
                  extent={{60,-40},{80,-80}},
                  lineColor={0,0,0},
                  fillPattern=FillPattern.VerticalCylinder,
                  fillColor={192,192,192}),
                Rectangle(
                  extent={{-10,80},{10,40}},
                  lineColor={0,0,0},
                  fillPattern=FillPattern.VerticalCylinder,
                  fillColor={192,192,192}),
                Line(
                  points={{-6,56},{6,64}},
                  color={0,0,0},
                  thickness=0.5),
                Ellipse(
                  extent={{-2,62},{2,58}},
                  lineColor={0,0,0},
                  fillColor={0,0,0},
                  fillPattern=FillPattern.Solid),
                Line(points={{-100,0},{-90,0},{-90,60},{0,60}}),
                Text(extent={{20,80},{100,60}}, textString=
                                                    "%name")}),
            Window(
              x=0.34,
              y=0.19,
              width=0.6,
              height=0.6),
            Documentation(info="This very simple model contains a throttle and a control volume which
can be connected to the intake system of an engine (or cylinder) in
order to simulate the effects of manifold filling and emptying.  A
throttle position of 90 degrees corresponds to wide open throttle (WOT)
while a throttle position of 0 degrees corresponds to a closed throttle.
"),         Diagram(coordinateSystem(preserveAspectRatio=true, extent={{-100,
                    -100},{100,100}},
                grid={2,2}),           graphics));
        end Manifold;

        model ValveExhaust "Engine poppet valve"
          parameter Modelica.SIunits.Diameter dia=0.012 "Valve diameter";
          parameter Modelica.SIunits.Length max_lift=0.012 "Maximum Valve Lift";
          parameter Real max_discharge=0.7 "Maximum Discharge Coefficient";
          extends Orifice(final Aref=Modelica.Constants.pi*(dia/2)^2);
        protected
          parameter Real c_over_l=max_discharge/max_lift;
        public
          Modelica.Mechanics.Translational.Interfaces.Flange_a lift annotation (
                                         layer="icon", Placement(
                transformation(extent={{-10,90},{10,110}}, rotation=0)));
        equation
          lift.f = 0;
          Cd = c_over_l*lift.s;
          // Cd = 0.0;
          annotation (
            Icon(coordinateSystem(
                preserveAspectRatio=false,
                extent={{-100,-100},{100,100}},
                grid={2,2}), graphics={
                Rectangle(
                  extent={{-40,-60},{40,-70}},
                  lineColor={0,0,0},
                  fillPattern=FillPattern.VerticalCylinder,
                  fillColor={192,192,192}),
                Polygon(
                  points={{-40,-60},{-28,-52},{30,-52},{40,-60},{-40,-60}},
                  lineColor={0,0,0},
                  fillPattern=FillPattern.VerticalCylinder,
                  fillColor={160,160,164}),
                Rectangle(
                  extent={{-10,90},{10,-52}},
                  lineColor={0,0,0},
                  fillPattern=FillPattern.VerticalCylinder,
                  fillColor={192,192,192}),
                Polygon(
                  points={{-40,-54},{-28,-46},{-28,-40},{-40,-30},{-60,-16},{
                      -100,-16},{-100,-100},{-60,-100},{-60,-70},{-60,-58},{
                      -40,-58},{-40,-54}},
                  lineColor={0,0,0},
                  fillColor={255,255,255},
                  fillPattern=FillPattern.Forward),
                Polygon(
                  points={{-100,40},{-60,40},{-10,16},{-10,100},{-100,100},{
                      -100,40}},
                  lineColor={0,0,0},
                  fillColor={255,255,255},
                  fillPattern=FillPattern.Forward),
                Polygon(
                  points={{10,6},{20,0},{30,-40},{30,-46},{40,-54},{40,-58},{
                      60,-58},{60,-100},{100,-100},{100,100},{10,100},{10,6}},
                  lineColor={0,0,0},
                  fillColor={255,255,255},
                  fillPattern=FillPattern.Forward),
                Text(extent={{12,64},{100,44}}, textString=
                                                    "%name")}),
            Window(
              x=0.39,
              y=0.31,
              width=0.6,
              height=0.6),
            Documentation(info="This is a very simple model of an engine valve the is derived from the \"Orifice\" model of isentropic flow.
The valve model must be connected to two different gas volumes (or reservoirs).  In addition, a translational
connector is used to represent the lift of the valve.
"),         Diagram(coordinateSystem(preserveAspectRatio=false,extent={{-100,
                    -100},{100,100}},
                grid={2,2}),           graphics));
        end ValveExhaust;

        model Combustion_Weibe "Simple combustion model"
          parameter Modelica.SIunits.SpecificEnergy lhv=44e+6
            "Lower heating value";
          parameter Real afr=14.6 "Stoichiometric Air/Fuel Ratio";
          parameter Types.Degrees burn_duration=60 "Duration of combustion";
          parameter Real m=2 "Combustion characterizing rate";
          Real tz;

        protected
          Modelica.SIunits.Energy amplitude;
          Modelica.SIunits.AngularVelocity w;
          Real dps;
          Modelica.SIunits.Time start_burn(start=-1);
          Modelica.SIunits.Time end_burn(                                                            start=-1);
          Boolean burning(start=false);
          Real tmp;
        public
          Interfaces.Gas cylinder annotation (Placement(transformation(extent={{-10,-50},
                    {10,-30}}, rotation=0)));
          Modelica.Blocks.Interfaces.RealInput mass         annotation (Placement(
                transformation(
                origin={-60,-110},
                extent={{10,-10},{-10,10}},
                rotation=270)));
          Modelica.Blocks.Interfaces.BooleanInput start             annotation (Placement(
                transformation(
                origin={0,110},
                extent={{-10,-10},{10,10}},
                rotation=270)));
          Modelica.Mechanics.Rotational.Interfaces.Flange_a crank annotation (Placement(
                transformation(extent={{-110,-50},{-90,-30}}, rotation=0)));
        equation
          assert(burn_duration > 1, "Invalid burn duration");
          cylinder.mdot = 0;
          w = der(crank.phi);
          dps = w*180/Modelica.Constants.pi;
          crank.tau = 0;
          tz=burn_duration/dps;
          cylinder.q= if (burning) then 
                -amplitude*6.908*(m+1)/(end_burn-start_burn)*((time-start_burn)/(end_burn-start_burn))^m*
                Modelica.Math.exp(-6.908*((time-start_burn)/(end_burn-start_burn))^(m+1)) else 0.0;
                // -amplitude*6.908*(m+1)/tz*((time)/tz)^m*
                // Modelica.Math.exp(-6.908*((time)/tz)^(m+1)) else 0.0;
          der(tmp) = cylinder.q;

        algorithm
          when start then
            start_burn := time;
            end_burn := time + (burn_duration/dps);
            amplitude := lhv*(mass/(afr + 1));
            burning := true;
          end when;
          when time >= end_burn then
            burning := false;
          end when;
          annotation (
            Window(
              x=0.41,
              y=0.28,
              width=0.26,
              height=0.6),
            Icon(coordinateSystem(
                preserveAspectRatio=false,
                extent={{-100,-100},{100,100}},
                grid={2,2}), graphics={
                Rectangle(
                  extent={{-8,40},{8,10}},
                  lineColor={0,0,0},
                  fillPattern=FillPattern.VerticalCylinder,
                  fillColor={192,192,192}),
                Polygon(
                  points={{4,10},{4,2},{-2,2},{-2,0},{6,0},{6,10},{4,10}},
                  lineColor={0,0,0},
                  fillColor={192,192,192},
                  fillPattern=FillPattern.Solid),
                Rectangle(
                  extent={{-2,10},{2,6}},
                  lineColor={0,0,0},
                  fillColor={160,160,164},
                  fillPattern=FillPattern.Solid),
                Polygon(
                  points={{-12,4},{-6,-2},{8,-2},{14,4},{22,4},{16,0},{20,-6},{12,-6},{10,
                      -12},{6,-6},{4,-18},{0,-8},{-6,-14},{-6,-6},{-16,-8},{-12,-2},{-20,
                      2},{-12,4}},
                  lineColor={255,255,0},
                  fillColor={255,255,0},
                  fillPattern=FillPattern.Solid),
                Polygon(
                  points={{-20,2},{-26,2},{-16,-2},{-20,-12},{-8,-10},{-10,-20},{-2,-16},
                      {6,-26},{8,-16},{12,-18},{16,-10},{26,-8},{22,-2},{30,4},{22,4},{16,
                      0},{20,-6},{12,-6},{10,-12},{6,-8},{4,-18},{0,-8},{-6,-14},{-6,-6},
                      {-14,-8},{-12,-2},{-20,2}},
                  lineColor={255,0,0},
                  fillColor={255,127,0},
                  fillPattern=FillPattern.Solid),
                Polygon(
                  points={{-80,-80},{-80,-20},{-28,0},{-28,10},{-8,10},{-8,40},{-92,40},
                      {-92,-80},{-80,-80}},
                  lineColor={0,0,0},
                  fillColor={255,255,255},
                  fillPattern=FillPattern.Forward),
                Polygon(
                  points={{8,40},{8,10},{32,10},{32,0},{80,-20},{80,-80},{90,-80},{90,40},
                      {8,40},{8,10},{8,40}},
                  lineColor={0,0,0},
                  fillColor={255,255,255},
                  fillPattern=FillPattern.Forward),
                Line(points={{0,112},{0,40}}, color={255,0,255}),
                Text(extent={{-60,-80},{60,-100}}, textString=
                                                       "%name")}),
            Documentation(info="This is a simplified combustion models.  This
model works by computing an instantaneous heat
release (i.e. energy given off as a result
of combustion) based on an idealized combustion
transient.  The duration of the burning is
typically determined by the motion of the
fuel/air mixture inside the cylinder but in
the case of this model it is assumed that
the burn duration is a fixed parameter.
"),         Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,
                    100}},
                grid={2,2}),           graphics));
        end Combustion_Weibe;

        model IndividualCylinder_Weibe
          "Collection of parts for a complete individual cylinder"
          extends Interfaces.Cylinder;
          parameter Types.Degrees spark_advance "Spark advance";
          parameter Types.Degrees burn_duration "Burn Duration";
          parameter Types.Degrees evo=40 "Exhaust Valve Opening";
          parameter Types.Degrees ivo=150 "Intake Valve Opening";
          parameter Types.Degrees evc=205 "Exhaust Valve Closing";
          parameter Types.Degrees ivc=310 "Intake Valve Closing";
          parameter Types.Degrees crank_shift=0 "Crankshaft Shift";
          parameter Modelica.SIunits.Diameter ivd=0.032 "Intake Valve Diameter";
          parameter Modelica.SIunits.Diameter evd=0.028
            "Exhaust Valve Diameter";
          Engine.Components.MasslessPiston piston annotation (
                        Placement(transformation(extent={{-10,-50},{10,-30}},
                  rotation=0)));
          Engine.Components.CrankSlider crankslider annotation (
                            Placement(transformation(extent={{-20,-120},{20,
                    -80}}, rotation=0)));
          Engine.Components.ControlVolume combustion_chamber annotation (Placement(
                transformation(extent={{-10,-20},{10,0}}, rotation=0)));
          Engine.Components.Valve intake_valve(dia=ivd) annotation (
                              Placement(transformation(extent={{-40,20},{-20,
                    40}}, rotation=0)));
          Engine.Components.Valve exhaust_valve(dia=evd) annotation (
                             Placement(transformation(extent={{40,20},{20,40}},
                  rotation=0)));
          Engine.Components.TimingBelt timing_belt annotation (Placement(
                transformation(extent={{-100,-60},{-60,-20}}, rotation=0)));
          Engine.Components.Cam intake_cam(vo=ivo, vc=ivc) annotation (
                                Placement(transformation(extent={{-60,60},{
                    -40,80}}, rotation=0)));
          Engine.Components.Cam exhaust_cam(vo=evo, vc=evc) annotation (
                               Placement(transformation(extent={{60,60},{40,
                    80}}, rotation=0)));
          Engine.Components.SparkControl spark_control(spark_advance=spark_advance) 
              annotation (                          Placement(transformation(
                  extent={{-24,44},{-4,64}}, rotation=0)));
          Engine.Components.ChamberVolume chamber_volume annotation (
                            Placement(transformation(extent={{56,-20},{76,0}},
                  rotation=0)));
          OffsetShaft offset_shaft(shift=crank_shift) annotation (
                                Placement(transformation(extent={{-20,-140},{
                    20,-180}}, rotation=0)));
          Combustion_Weibe combustion_Weibe 
            annotation (Placement(transformation(extent={{-10,22},{10,42}})));
        equation
          connect(crankslider.piston, chamber_volume.piston) annotation (Line(
              points={{0,-80},{66,-80},{66,-18}},
              color={0,127,0},
              smooth=Smooth.None));
          connect(crankslider.crank, offset_shaft.cyl) annotation (Line(
              points={{0,-108},{0,-140}},
              color={0,0,0},
              smooth=Smooth.None));
          connect(crankslider.crank, timing_belt.crankshaft) annotation (Line(
              points={{0,-108},{-80,-108},{-80,-56}},
              color={0,0,0},
              smooth=Smooth.None));
          connect(crankslider.crank, spark_control.crank) annotation (Line(
              points={{0,-108},{-14,-108},{-14,44}},
              color={0,0,0},
              smooth=Smooth.None));
          connect(offset_shaft.crank, crankshaft) annotation (Line(
              points={{-20,-160},{-20,-198},{0,-198}},
              color={0,0,0},
              smooth=Smooth.None));
          connect(timing_belt.camshaft, exhaust_cam.camshaft) annotation (Line(
              points={{-80,-28},{-80,86},{60,86},{60,70},{52.8,70}},
              color={0,0,0},
              smooth=Smooth.None));
          connect(timing_belt.camshaft, intake_cam.camshaft) annotation (Line(
              points={{-80,-28},{-80,70},{-52.8,70}},
              color={0,0,0},
              smooth=Smooth.None));
          connect(intake_valve.a, intake) annotation (Line(
              points={{-40,30},{-100,30},{-100,80}},
              color={255,0,0},
              smooth=Smooth.None));
          connect(exhaust_valve.a, exhaust) annotation (Line(
              points={{40,30},{100,30},{100,80}},
              color={255,0,0},
              smooth=Smooth.None));
          connect(intake_cam.valve_lift, intake_valve.lift) annotation (Line(
              points={{-40,70},{-40,62.5},{-20,62.5},{-20,55},{-30,55},{-30,
                  40}},
              color={0,127,0},
              smooth=Smooth.None));
          connect(exhaust_cam.valve_lift, exhaust_valve.lift) annotation (Line(
              points={{40,70},{40,62.5},{24,62.5},{24,55},{30,55},{30,40}},
              color={0,127,0},
              smooth=Smooth.None));
          connect(combustion_chamber.state, piston.chamber) annotation (Line(
              points={{0,-10},{0,-30}},
              color={255,0,0},
              smooth=Smooth.None));
          connect(piston.piston, crankslider.piston) annotation (Line(
              points={{0,-42.2},{0,-80}},
              color={0,127,0},
              smooth=Smooth.None));
          connect(chamber_volume.volume, combustion_chamber.volume) annotation (
             Line(
              points={{55,-10},{11,-10}},
              color={0,0,127},
              smooth=Smooth.None));
          connect(combustion_chamber.state, exhaust_valve.b) annotation (Line(
              points={{0,-10},{6,-10},{6,12},{30,12},{30,20}},
              color={255,0,0},
              smooth=Smooth.None));
          connect(combustion_chamber.state, intake_valve.b) annotation (Line(
              points={{0,-10},{-12,-10},{-12,12},{-30,12},{-30,20}},
              color={255,0,0},
              smooth=Smooth.None));
          connect(geom, chamber_volume.geom) annotation (Line(
              points={{110,-50},{80,-50},{80,-10},{77,-10}},
              color={0,0,0},
              smooth=Smooth.None));
          connect(geom, piston.geom) annotation (Line(
              points={{110,-50},{80,-50},{80,-40},{11,-40}},
              color={0,0,0},
              smooth=Smooth.None));
          connect(geom, crankslider.geom) annotation (Line(
              points={{110,-50},{80,-50},{80,-100},{22,-100}},
              color={0,0,0},
              smooth=Smooth.None));
          connect(spark_control.spark, combustion_Weibe.start) annotation (Line(
              points={{-3,54},{0,54},{0,43}},
              color={255,0,255},
              smooth=Smooth.None));
          connect(combustion_Weibe.cylinder, combustion_chamber.state) 
            annotation (Line(
              points={{0,28},{0,-10}},
              color={255,0,0},
              smooth=Smooth.None));
          connect(combustion_Weibe.mass, combustion_chamber.mass) annotation (
              Line(
              points={{-6,21},{-6,1}},
              color={0,0,127},
              smooth=Smooth.None));
          connect(combustion_Weibe.crank, crankslider.crank) annotation (Line(
              points={{-10,28},{-22,28},{-22,-108},{0,-108}},
              color={0,0,0},
              smooth=Smooth.None));
          annotation (
            Window(
              x=0.07,
              y=0.03,
              width=0.99,
              height=0.91),
            Icon(coordinateSystem(
                preserveAspectRatio=false,
                extent={{-100,-200},{100,100}},
                grid={2,2}), graphics={
                Polygon(
                  points={{-60,-140},{-60,40},{-42,40},{-42,44},{-40,46},{-60,
                      60},{-100,60},{-100,-140},{-60,-140}},
                  lineColor={0,0,0},
                  fillColor={192,192,192},
                  fillPattern=FillPattern.Solid),
                Polygon(
                  points={{-100,96},{-60,96},{-40,80},{-20,60},{-20,46},{-18,
                      44},{-18,40},{18,40},{18,44},{20,46},{20,60},{40,80},{
                      60,96},{100,96},{100,100},{-100,100},{-100,96}},
                  lineColor={0,0,0},
                  fillColor={192,192,192},
                  fillPattern=FillPattern.Solid),
                Rectangle(
                  extent={{-58,12},{58,-64}},
                  lineColor={0,0,0},
                  fillPattern=FillPattern.VerticalCylinder,
                  fillColor={192,192,192}),
                Rectangle(
                  extent={{-60,6},{60,0}},
                  lineColor={0,0,0},
                  fillColor={0,0,0},
                  fillPattern=FillPattern.Solid),
                Rectangle(
                  extent={{-60,-8},{60,-14}},
                  lineColor={0,0,0},
                  fillColor={0,0,0},
                  fillPattern=FillPattern.Solid),
                Rectangle(
                  extent={{-60,-20},{60,-26}},
                  lineColor={0,0,0},
                  fillColor={0,0,0},
                  fillPattern=FillPattern.Solid),
                Polygon(
                  points={{-58,-64},{-40,-52},{40,-52},{58,-64},{-58,-64}},
                  lineColor={0,0,255},
                  pattern=LinePattern.None,
                  fillColor={255,255,255},
                  fillPattern=FillPattern.Solid),
                Ellipse(
                  extent={{-4,-34},{4,-42}},
                  lineColor={0,0,0},
                  fillColor={0,0,0},
                  fillPattern=FillPattern.Solid),
                Ellipse(extent={{-40,-240},{40,-160}}, lineColor={192,192,192}),
                Line(
                  points={{0,-200},{30,-174},{0,-38}},
                  color={0,0,0},
                  thickness=1),
                Rectangle(
                  extent={{-42,36},{-18,32}},
                  lineColor={0,0,0},
                  fillPattern=FillPattern.VerticalCylinder,
                  fillColor={192,192,192}),
                Polygon(
                  points={{-42,36},{-40,38},{-20,38},{-18,36},{-42,36}},
                  lineColor={160,160,164},
                  fillColor={192,192,192},
                  fillPattern=FillPattern.Solid),
                Rectangle(
                  extent={{-32,38},{-28,96}},
                  lineColor={0,0,0},
                  fillPattern=FillPattern.VerticalCylinder,
                  fillColor={192,192,192}),
                Polygon(
                  points={{18,44},{20,46},{40,46},{42,44},{18,44}},
                  lineColor={160,160,164},
                  fillColor={192,192,192},
                  fillPattern=FillPattern.Solid),
                Rectangle(
                  extent={{18,44},{42,40}},
                  lineColor={0,0,0},
                  fillPattern=FillPattern.VerticalCylinder,
                  fillColor={192,192,192}),
                Rectangle(
                  extent={{28,46},{32,96}},
                  lineColor={0,0,0},
                  fillPattern=FillPattern.VerticalCylinder,
                  fillColor={192,192,192}),
                Polygon(
                  points={{100,-140},{100,60},{60,60},{40,46},{42,44},{42,40},
                      {60,40},{60,-140},{100,-140}},
                  lineColor={0,0,0},
                  fillColor={192,192,192},
                  fillPattern=FillPattern.Solid),
                Text(extent={{-60,-100},{60,-120}}, textString=
                                                        "%name")}),
            Documentation(info="All the basic componets of a single cylinder, 2-valve engine
have been compiled in this model.
"),         Diagram(coordinateSystem(preserveAspectRatio=false,extent={{-100,
                    -200},{100,100}},
                grid={2,2}),           graphics));
        end IndividualCylinder_Weibe;

        model HeatTransfer_Hohenberg
          "Heat Transfer of the cylinder by using Hohenberg's equation"
          // Modelica.SIunits.Volume Vc=volume "Chamber volume";

          // Modelica.SIunits.Temperature Tg=state.T "Gas Temperature";

        Modelica.SIunits.Temp_C Tw = 160 "Cylinder Wall Temperature";

        // Real hloss "Coefficient Of Heat Transfer";
        // Real Cm "Pistion Velocity";
        // Real Aloss "Heat transfer Area";
          // Modelica.SIunits.Heat qloss "Heat transfer";

        protected
          Modelica.SIunits.CoefficientOfHeatTransfer hloss
            "Coefficient Of Heat Transfer";
          Modelica.SIunits.Velocity Cm "Pistion Velocity";
          Modelica.SIunits.Area Aloss "Heat transfer Area";

        public
          Modelica.Blocks.Interfaces.RealInput volume 
            annotation (Placement(transformation(extent={{120,64},{100,44}}),
                iconTransformation(extent={{112,56},{100,44}})));
          Modelica.Mechanics.Translational.Interfaces.Flange_a piston annotation (
                                         Placement(transformation(extent={{-8,90},
                    {12,110}}, rotation=0)));
          Interfaces.EngineGeometryRequired geom annotation (Placement(
                transformation(
                origin={110,0},
                extent={{-10,-10},{10,10}},
                rotation=180)));
          Modelica.Blocks.Interfaces.RealOutput qloss annotation (Placement(
                transformation(
                extent={{-10,-10},{10,10}},
                rotation=90,
                origin={0,110})));

        public
          Interfaces.Gas state "Gas state" annotation (
              layer="icon",
            Placement(transformation(extent={{-10,-10},{10,10}}, rotation=0)));
        equation
          Cm=abs(der(piston.s));
          hloss=130*(state.P/100000)^0.8*(Cm+1.4)^0.8/(volume^0.06*(state.T+273.15)^0.4);
          Aloss=Modelica.Constants.pi*geom.bore*(geom.bore/2+piston.s);
          qloss=hloss*Aloss*(state.T-Tw);
          piston.f=0;
          state.mdot=0;
          state.q=0;

          connect(piston, piston) annotation (Line(
              points={{2,100},{2,100}},
              color={0,127,0},
              smooth=Smooth.None));
          annotation (Icon(coordinateSystem(preserveAspectRatio=true, extent={{-100,
                    -100},{100,100}}), graphics), Diagram(graphics));
        end HeatTransfer_Hohenberg;

        model IndividualCylinder_Weibe_Hohenberg
          "Collection of parts for a complete individual cylinder"
          extends Interfaces.Cylinder;
          parameter Types.Degrees spark_advance "Spark advance";
          parameter Types.Degrees burn_duration "Burn Duration";
          parameter Types.Degrees evo=40 "Exhaust Valve Opening";
          parameter Types.Degrees ivo=150 "Intake Valve Opening";
          parameter Types.Degrees evc=205 "Exhaust Valve Closing";
          parameter Types.Degrees ivc=310 "Intake Valve Closing";
          parameter Types.Degrees crank_shift=0 "Crankshaft Shift";
          parameter Modelica.SIunits.Diameter ivd=0.032 "Intake Valve Diameter";
          parameter Modelica.SIunits.Diameter evd=0.028
            "Exhaust Valve Diameter";
          Engine.Components.MasslessPiston piston annotation (
                        Placement(transformation(extent={{-10,-50},{10,-30}},
                  rotation=0)));
          Engine.Components.CrankSlider crankslider annotation (
                            Placement(transformation(extent={{-20,-120},{20,
                    -80}}, rotation=0)));
          Engine.Components.ControlVolume combustion_chamber annotation (Placement(
                transformation(extent={{-10,-20},{10,0}}, rotation=0)));
          Engine.Components.Valve intake_valve(dia=ivd) annotation (
                              Placement(transformation(extent={{-40,20},{-20,
                    40}}, rotation=0)));
          Engine.Components.Valve exhaust_valve(dia=evd) annotation (
                             Placement(transformation(extent={{40,20},{20,40}},
                  rotation=0)));
          Engine.Components.TimingBelt timing_belt annotation (Placement(
                transformation(extent={{-100,-60},{-60,-20}}, rotation=0)));
          Engine.Components.Cam intake_cam(vo=ivo, vc=ivc) annotation (
                                Placement(transformation(extent={{-60,60},{
                    -40,80}}, rotation=0)));
          Engine.Components.Cam exhaust_cam(vo=evo, vc=evc) annotation (
                               Placement(transformation(extent={{60,60},{40,
                    80}}, rotation=0)));
          Engine.Components.SparkControl spark_control(spark_advance=spark_advance) 
              annotation (                          Placement(transformation(
                  extent={{-24,44},{-4,64}}, rotation=0)));
          Engine.Components.ChamberVolume chamber_volume annotation (
                            Placement(transformation(extent={{56,-20},{76,0}},
                  rotation=0)));
          OffsetShaft offset_shaft(shift=crank_shift) annotation (
                                Placement(transformation(extent={{-20,-140},{
                    20,-180}}, rotation=0)));
          HeatTransfer_Hohenberg heatTransfer 
                                    annotation (Placement(transformation(extent={{-50,-32},
                    {-30,-12}})));
          Combustion_Weibe_Hohenberg combustion_Weibe_Hohenberg 
            annotation (Placement(transformation(extent={{-10,18},{10,38}})));
        equation
          connect(crankslider.piston, chamber_volume.piston) annotation (Line(
              points={{0,-80},{66,-80},{66,-18}},
              color={0,127,0},
              smooth=Smooth.None));
          connect(crankslider.crank, offset_shaft.cyl) annotation (Line(
              points={{0,-108},{0,-140}},
              color={0,0,0},
              smooth=Smooth.None));
          connect(crankslider.crank, timing_belt.crankshaft) annotation (Line(
              points={{0,-108},{-80,-108},{-80,-56}},
              color={0,0,0},
              smooth=Smooth.None));
          connect(crankslider.crank, spark_control.crank) annotation (Line(
              points={{0,-108},{-14,-108},{-14,44}},
              color={0,0,0},
              smooth=Smooth.None));
          connect(offset_shaft.crank, crankshaft) annotation (Line(
              points={{-20,-160},{-20,-198},{0,-198}},
              color={0,0,0},
              smooth=Smooth.None));
          connect(timing_belt.camshaft, exhaust_cam.camshaft) annotation (Line(
              points={{-80,-28},{-80,86},{60,86},{60,70},{52.8,70}},
              color={0,0,0},
              smooth=Smooth.None));
          connect(timing_belt.camshaft, intake_cam.camshaft) annotation (Line(
              points={{-80,-28},{-80,70},{-52.8,70}},
              color={0,0,0},
              smooth=Smooth.None));
          connect(intake_valve.a, intake) annotation (Line(
              points={{-40,30},{-100,30},{-100,80}},
              color={255,0,0},
              smooth=Smooth.None));
          connect(exhaust_valve.a, exhaust) annotation (Line(
              points={{40,30},{100,30},{100,80}},
              color={255,0,0},
              smooth=Smooth.None));
          connect(intake_cam.valve_lift, intake_valve.lift) annotation (Line(
              points={{-40,70},{-40,62.5},{-20,62.5},{-20,55},{-30,55},{-30,
                  40}},
              color={0,127,0},
              smooth=Smooth.None));
          connect(exhaust_cam.valve_lift, exhaust_valve.lift) annotation (Line(
              points={{40,70},{40,62.5},{24,62.5},{24,55},{30,55},{30,40}},
              color={0,127,0},
              smooth=Smooth.None));
          connect(combustion_chamber.state, piston.chamber) annotation (Line(
              points={{0,-10},{0,-30}},
              color={255,0,0},
              smooth=Smooth.None));
          connect(piston.piston, crankslider.piston) annotation (Line(
              points={{0,-42.2},{0,-80}},
              color={0,127,0},
              smooth=Smooth.None));
          connect(chamber_volume.volume, combustion_chamber.volume) annotation (
             Line(
              points={{55,-10},{11,-10}},
              color={0,0,127},
              smooth=Smooth.None));
          connect(combustion_chamber.state, exhaust_valve.b) annotation (Line(
              points={{0,-10},{6,-10},{6,12},{30,12},{30,20}},
              color={255,0,0},
              smooth=Smooth.None));
          connect(combustion_chamber.state, intake_valve.b) annotation (Line(
              points={{0,-10},{-12,-10},{-12,12},{-30,12},{-30,20}},
              color={255,0,0},
              smooth=Smooth.None));
          connect(geom, chamber_volume.geom) annotation (Line(
              points={{110,-50},{80,-50},{80,-10},{77,-10}},
              color={0,0,0},
              smooth=Smooth.None));
          connect(geom, piston.geom) annotation (Line(
              points={{110,-50},{80,-50},{80,-40},{11,-40}},
              color={0,0,0},
              smooth=Smooth.None));
          connect(geom, crankslider.geom) annotation (Line(
              points={{110,-50},{80,-50},{80,-100},{22,-100}},
              color={0,0,0},
              smooth=Smooth.None));
          connect(crankslider.piston, heatTransfer.piston) annotation (Line(
              points={{0,-80},{-39.8,-80},{-39.8,-12}},
              color={0,127,0},
              smooth=Smooth.None));
          connect(heatTransfer.geom, geom) annotation (Line(
              points={{-29,-22},{80,-22},{80,-50},{110,-50}},
              color={0,0,0},
              smooth=Smooth.None));
          connect(chamber_volume.volume, heatTransfer.volume) annotation (Line(
              points={{55,-10},{40,-10},{40,-17},{-29.4,-17}},
              color={0,0,127},
              smooth=Smooth.None));
          connect(combustion_chamber.state, heatTransfer.state) annotation (
              Line(
              points={{0,-10},{-40,-10},{-40,-22}},
              color={255,0,0},
              smooth=Smooth.None));
          connect(combustion_Weibe_Hohenberg.cylinder, combustion_chamber.state) 
            annotation (Line(
              points={{0,24},{0,-10}},
              color={255,0,0},
              smooth=Smooth.None));
          connect(crankslider.crank, combustion_Weibe_Hohenberg.crank) 
            annotation (Line(
              points={{0,-108},{-22,-108},{-22,24},{-10,24}},
              color={0,0,0},
              smooth=Smooth.None));
          connect(spark_control.spark, combustion_Weibe_Hohenberg.start) 
            annotation (Line(
              points={{-3,54},{0,54},{0,39}},
              color={255,0,255},
              smooth=Smooth.None));
          connect(heatTransfer.qloss, combustion_Weibe_Hohenberg.qloss) 
            annotation (Line(
              points={{-40,-11},{-40,68},{14,68},{14,17},{8,17}},
              color={0,0,127},
              smooth=Smooth.None));
          connect(combustion_Weibe_Hohenberg.mass, combustion_chamber.mass) 
            annotation (Line(
              points={{-6,17},{-6,1}},
              color={0,0,127},
              smooth=Smooth.None));
          annotation (
            Window(
              x=0.07,
              y=0.03,
              width=0.99,
              height=0.91),
            Icon(coordinateSystem(
                preserveAspectRatio=false,
                extent={{-100,-200},{100,100}},
                grid={2,2}), graphics={
                Polygon(
                  points={{-60,-140},{-60,40},{-42,40},{-42,44},{-40,46},{-60,
                      60},{-100,60},{-100,-140},{-60,-140}},
                  lineColor={0,0,0},
                  fillColor={192,192,192},
                  fillPattern=FillPattern.Solid),
                Polygon(
                  points={{-100,96},{-60,96},{-40,80},{-20,60},{-20,46},{-18,
                      44},{-18,40},{18,40},{18,44},{20,46},{20,60},{40,80},{
                      60,96},{100,96},{100,100},{-100,100},{-100,96}},
                  lineColor={0,0,0},
                  fillColor={192,192,192},
                  fillPattern=FillPattern.Solid),
                Rectangle(
                  extent={{-58,12},{58,-64}},
                  lineColor={0,0,0},
                  fillPattern=FillPattern.VerticalCylinder,
                  fillColor={192,192,192}),
                Rectangle(
                  extent={{-60,6},{60,0}},
                  lineColor={0,0,0},
                  fillColor={0,0,0},
                  fillPattern=FillPattern.Solid),
                Rectangle(
                  extent={{-60,-8},{60,-14}},
                  lineColor={0,0,0},
                  fillColor={0,0,0},
                  fillPattern=FillPattern.Solid),
                Rectangle(
                  extent={{-60,-20},{60,-26}},
                  lineColor={0,0,0},
                  fillColor={0,0,0},
                  fillPattern=FillPattern.Solid),
                Polygon(
                  points={{-58,-64},{-40,-52},{40,-52},{58,-64},{-58,-64}},
                  lineColor={0,0,255},
                  pattern=LinePattern.None,
                  fillColor={255,255,255},
                  fillPattern=FillPattern.Solid),
                Ellipse(
                  extent={{-4,-34},{4,-42}},
                  lineColor={0,0,0},
                  fillColor={0,0,0},
                  fillPattern=FillPattern.Solid),
                Ellipse(extent={{-40,-240},{40,-160}}, lineColor={192,192,192}),
                Line(
                  points={{0,-200},{30,-174},{0,-38}},
                  color={0,0,0},
                  thickness=1),
                Rectangle(
                  extent={{-42,36},{-18,32}},
                  lineColor={0,0,0},
                  fillPattern=FillPattern.VerticalCylinder,
                  fillColor={192,192,192}),
                Polygon(
                  points={{-42,36},{-40,38},{-20,38},{-18,36},{-42,36}},
                  lineColor={160,160,164},
                  fillColor={192,192,192},
                  fillPattern=FillPattern.Solid),
                Rectangle(
                  extent={{-32,38},{-28,96}},
                  lineColor={0,0,0},
                  fillPattern=FillPattern.VerticalCylinder,
                  fillColor={192,192,192}),
                Polygon(
                  points={{18,44},{20,46},{40,46},{42,44},{18,44}},
                  lineColor={160,160,164},
                  fillColor={192,192,192},
                  fillPattern=FillPattern.Solid),
                Rectangle(
                  extent={{18,44},{42,40}},
                  lineColor={0,0,0},
                  fillPattern=FillPattern.VerticalCylinder,
                  fillColor={192,192,192}),
                Rectangle(
                  extent={{28,46},{32,96}},
                  lineColor={0,0,0},
                  fillPattern=FillPattern.VerticalCylinder,
                  fillColor={192,192,192}),
                Polygon(
                  points={{100,-140},{100,60},{60,60},{40,46},{42,44},{42,40},
                      {60,40},{60,-140},{100,-140}},
                  lineColor={0,0,0},
                  fillColor={192,192,192},
                  fillPattern=FillPattern.Solid),
                Text(extent={{-60,-100},{60,-120}}, textString=
                                                        "%name")}),
            Documentation(info="All the basic componets of a single cylinder, 2-valve engine
have been compiled in this model.
"),         Diagram(coordinateSystem(preserveAspectRatio=true, extent={{-100,
                    -200},{100,100}},
                grid={2,2}),           graphics));
        end IndividualCylinder_Weibe_Hohenberg;

        model Combustion_Weibe_Hohenberg "Simple combustion model"
          parameter Modelica.SIunits.SpecificEnergy lhv=44e+6
            "Lower heating value";
          parameter Real afr=14.6 "Stoichiometric Air/Fuel Ratio";
          parameter Types.Degrees burn_duration=60 "Duration of combustion";
          parameter Real m=2 "Combustion characterizing rate";
          Real tz;

        protected
          Modelica.SIunits.Energy amplitude;
          Modelica.SIunits.AngularVelocity w;
          Real dps;
          Modelica.SIunits.Time start_burn(start=-1);
          Modelica.SIunits.Time end_burn(start=-1);
          Boolean burning(start=false);
          Real tmp;
        public
          Interfaces.Gas cylinder annotation (Placement(transformation(extent=
                   {{-10,-50},{10,-30}}, rotation=0)));
          Modelica.Blocks.Interfaces.RealInput mass         annotation (Placement(
                transformation(
                origin={-60,-110},
                extent={{10,-10},{-10,10}},
                rotation=270)));
          Modelica.Blocks.Interfaces.BooleanInput start             annotation (Placement(
                transformation(
                origin={0,110},
                extent={{-10,-10},{10,10}},
                rotation=270)));
          Modelica.Mechanics.Rotational.Interfaces.Flange_a crank annotation (Placement(
                transformation(extent={{-110,-50},{-90,-30}}, rotation=0)));
          Modelica.Blocks.Interfaces.RealInput qloss annotation (Placement(
                transformation(
                extent={{0,0},{20,20}},
                rotation=90,
                origin={70,-120})));

        equation
          assert(burn_duration > 1, "Invalid burn duration");
          cylinder.mdot = 0;
          w = der(crank.phi);
          dps = w*180/Modelica.Constants.pi;
          crank.tau = 0;
          tz=burn_duration/dps;
          cylinder.q= if (burning) then 
                -amplitude*6.908*(m+1)/(end_burn-start_burn)*((time-start_burn)/(end_burn-start_burn))^m*
                Modelica.Math.exp(-6.908*((time-start_burn)/(end_burn-start_burn))^(m+1))+qloss else 0.0;
                // -amplitude*6.908*(m+1)/tz*((time)/tz)^m*
                // Modelica.Math.exp(-6.908*((time)/tz)^(m+1)) else 0.0;
          der(tmp) = cylinder.q;

        algorithm
          when start then
            start_burn := time;
            end_burn := time + (burn_duration/dps);
            amplitude := lhv*(mass/(afr + 1));
            burning := true;
          end when;
          when time >= end_burn then
            burning := false;
          end when;
          annotation (
            Window(
              x=0.41,
              y=0.28,
              width=0.26,
              height=0.6),
            Icon(coordinateSystem(
                preserveAspectRatio=false,
                extent={{-100,-100},{100,100}},
                grid={2,2}), graphics={
                Rectangle(
                  extent={{-8,40},{8,10}},
                  lineColor={0,0,0},
                  fillPattern=FillPattern.VerticalCylinder,
                  fillColor={192,192,192}),
                Polygon(
                  points={{4,10},{4,2},{-2,2},{-2,0},{6,0},{6,10},{4,10}},
                  lineColor={0,0,0},
                  fillColor={192,192,192},
                  fillPattern=FillPattern.Solid),
                Rectangle(
                  extent={{-2,10},{2,6}},
                  lineColor={0,0,0},
                  fillColor={160,160,164},
                  fillPattern=FillPattern.Solid),
                Polygon(
                  points={{-12,4},{-6,-2},{8,-2},{14,4},{22,4},{16,0},{20,-6},
                      {12,-6},{10,-12},{6,-6},{4,-18},{0,-8},{-6,-14},{-6,-6},
                      {-16,-8},{-12,-2},{-20,2},{-12,4}},
                  lineColor={255,255,0},
                  fillColor={255,255,0},
                  fillPattern=FillPattern.Solid),
                Polygon(
                  points={{-20,2},{-26,2},{-16,-2},{-20,-12},{-8,-10},{-10,
                      -20},{-2,-16},{6,-26},{8,-16},{12,-18},{16,-10},{26,-8},
                      {22,-2},{30,4},{22,4},{16,0},{20,-6},{12,-6},{10,-12},{
                      6,-8},{4,-18},{0,-8},{-6,-14},{-6,-6},{-14,-8},{-12,-2},
                      {-20,2}},
                  lineColor={255,0,0},
                  fillColor={255,127,0},
                  fillPattern=FillPattern.Solid),
                Polygon(
                  points={{-80,-80},{-80,-20},{-28,0},{-28,10},{-8,10},{-8,40},
                      {-92,40},{-92,-80},{-80,-80}},
                  lineColor={0,0,0},
                  fillColor={255,255,255},
                  fillPattern=FillPattern.Forward),
                Polygon(
                  points={{8,40},{8,10},{32,10},{32,0},{80,-20},{80,-80},{90,
                      -80},{90,40},{8,40},{8,10},{8,40}},
                  lineColor={0,0,0},
                  fillColor={255,255,255},
                  fillPattern=FillPattern.Forward),
                Line(points={{0,112},{0,40}}, color={255,0,255}),
                Text(extent={{-60,-80},{60,-100}}, textString=
                                                       "%name")}),
            Documentation(info="This is a simplified combustion models.  This
model works by computing an instantaneous heat
release (i.e. energy given off as a result
of combustion) based on an idealized combustion
transient.  The duration of the burning is
typically determined by the motion of the
fuel/air mixture inside the cylinder but in
the case of this model it is assumed that
the burn duration is a fixed parameter.
"),         Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,
                    -100},{100,100}},
                grid={2,2}),           graphics));
        end Combustion_Weibe_Hohenberg;

        model Combustion_Weibe_Hohenberg_Biogas "Simple combustion model"
          parameter Types.Degrees burn_duration=60 "Duration of combustion";
          parameter Real m=2.2 "Combustion characterizing rate";
          parameter Real faia=1.43 "Accessive air rate";
          Real tz;

        protected
          Modelica.SIunits.Energy amplitude;
          Modelica.SIunits.AngularVelocity w;
          Real dps;
          Modelica.SIunits.Time start_burn(start=-1);
          Modelica.SIunits.Time end_burn(start=-1);
          Boolean burning(start=false);
          Real tmp;
        public
          Interfaces.Gas cylinder annotation (Placement(transformation(extent=
                   {{-10,-50},{10,-30}}, rotation=0)));
          Modelica.Blocks.Interfaces.RealInput mass         annotation (Placement(
                transformation(
                origin={-60,-110},
                extent={{10,-10},{-10,10}},
                rotation=270)));
          Modelica.Blocks.Interfaces.BooleanInput start             annotation (Placement(
                transformation(
                origin={0,110},
                extent={{-10,-10},{10,10}},
                rotation=270)));
          Modelica.Mechanics.Rotational.Interfaces.Flange_a crank annotation (Placement(
                transformation(extent={{-110,-50},{-90,-30}}, rotation=0)));
          Modelica.Blocks.Interfaces.RealInput qloss annotation (Placement(
                transformation(
                extent={{0,0},{20,20}},
                rotation=90,
                origin={70,-120})));

          Interfaces.BiogasPropertyRequired bioP "biogas Property Required" 
            annotation (Placement(transformation(
                extent={{-10,-10},{10,10}},
                rotation=180,
                origin={110,-6})));
        equation
          assert(burn_duration > 1, "Invalid burn duration");
          cylinder.mdot = 0;
          w = der(crank.phi);
          dps = w*180/Modelica.Constants.pi;
          crank.tau = 0;
          tz=burn_duration/dps;
          cylinder.q= if (burning) then 
                -amplitude*6.908*(m+1)/(end_burn-start_burn)*((time-start_burn)/(end_burn-start_burn))^m*
                Modelica.Math.exp(-6.908*((time-start_burn)/(end_burn-start_burn))^(m+1))+qloss else 0.0;
                // -amplitude*6.908*(m+1)/tz*((time)/tz)^m*
                // Modelica.Math.exp(-6.908*((time)/tz)^(m+1)) else 0.0;
          der(tmp) = cylinder.q;

        algorithm
          when start then
            start_burn := time;
            end_burn := time + (burn_duration/dps);
            amplitude := bioP.lhv*mass/(bioP.afr*faia + 1);
            burning := true;
          end when;
          when time >= end_burn then
            burning := false;
          end when;
          annotation (
            Window(
              x=0.41,
              y=0.28,
              width=0.26,
              height=0.6),
            Icon(coordinateSystem(
                preserveAspectRatio=false,
                extent={{-100,-100},{100,100}},
                grid={2,2}), graphics={
                Rectangle(
                  extent={{-8,40},{8,10}},
                  lineColor={0,0,0},
                  fillPattern=FillPattern.VerticalCylinder,
                  fillColor={192,192,192}),
                Polygon(
                  points={{4,10},{4,2},{-2,2},{-2,0},{6,0},{6,10},{4,10}},
                  lineColor={0,0,0},
                  fillColor={192,192,192},
                  fillPattern=FillPattern.Solid),
                Rectangle(
                  extent={{-2,10},{2,6}},
                  lineColor={0,0,0},
                  fillColor={160,160,164},
                  fillPattern=FillPattern.Solid),
                Polygon(
                  points={{-12,4},{-6,-2},{8,-2},{14,4},{22,4},{16,0},{20,-6},
                      {12,-6},{10,-12},{6,-6},{4,-18},{0,-8},{-6,-14},{-6,-6},
                      {-16,-8},{-12,-2},{-20,2},{-12,4}},
                  lineColor={255,255,0},
                  fillColor={255,255,0},
                  fillPattern=FillPattern.Solid),
                Polygon(
                  points={{-20,2},{-26,2},{-16,-2},{-20,-12},{-8,-10},{-10,
                      -20},{-2,-16},{6,-26},{8,-16},{12,-18},{16,-10},{26,-8},
                      {22,-2},{30,4},{22,4},{16,0},{20,-6},{12,-6},{10,-12},{
                      6,-8},{4,-18},{0,-8},{-6,-14},{-6,-6},{-14,-8},{-12,-2},
                      {-20,2}},
                  lineColor={255,0,0},
                  fillColor={255,127,0},
                  fillPattern=FillPattern.Solid),
                Polygon(
                  points={{-80,-80},{-80,-20},{-28,0},{-28,10},{-8,10},{-8,40},
                      {-92,40},{-92,-80},{-80,-80}},
                  lineColor={0,0,0},
                  fillColor={255,255,255},
                  fillPattern=FillPattern.Forward),
                Polygon(
                  points={{8,40},{8,10},{32,10},{32,0},{80,-20},{80,-80},{90,
                      -80},{90,40},{8,40},{8,10},{8,40}},
                  lineColor={0,0,0},
                  fillColor={255,255,255},
                  fillPattern=FillPattern.Forward),
                Line(points={{0,112},{0,40}}, color={255,0,255}),
                Text(extent={{-60,-80},{60,-100}}, textString=
                                                       "%name")}),
            Documentation(info="This is a simplified combustion models.  This
model works by computing an instantaneous heat
release (i.e. energy given off as a result
of combustion) based on an idealized combustion
transient.  The duration of the burning is
typically determined by the motion of the
fuel/air mixture inside the cylinder but in
the case of this model it is assumed that
the burn duration is a fixed parameter.
"),         Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,
                    -100},{100,100}},
                grid={2,2}),           graphics));
        end Combustion_Weibe_Hohenberg_Biogas;

        model IndividualCylinder_Weibe_Hohenberg_Biogas
          "Collection of parts for a complete individual cylinder"
          extends Interfaces.Cylinder;
          parameter Types.Degrees spark_advance "Spark advance";
          parameter Types.Degrees burn_duration "Burn Duration";
          parameter Types.Degrees evo=40 "Exhaust Valve Opening";
          parameter Types.Degrees ivo=150 "Intake Valve Opening";
          parameter Types.Degrees evc=205 "Exhaust Valve Closing";
          parameter Types.Degrees ivc=310 "Intake Valve Closing";
          parameter Types.Degrees crank_shift=0 "Crankshaft Shift";
          parameter Modelica.SIunits.Diameter ivd=0.032 "Intake Valve Diameter";
          parameter Modelica.SIunits.Diameter evd=0.028
            "Exhaust Valve Diameter";
          Engine.Components.MasslessPiston piston annotation (
                        Placement(transformation(extent={{-10,-50},{10,-30}},
                  rotation=0)));
          Engine.Components.CrankSlider crankslider annotation (
                            Placement(transformation(extent={{-20,-120},{20,
                    -80}}, rotation=0)));
          Engine.Components.ControlVolume combustion_chamber annotation (Placement(
                transformation(extent={{-10,-20},{10,0}}, rotation=0)));
          Engine.Components.Valve intake_valve(dia=ivd) annotation (
                              Placement(transformation(extent={{-40,20},{-20,
                    40}}, rotation=0)));
          Engine.Components.Valve exhaust_valve(dia=evd) annotation (
                             Placement(transformation(extent={{40,20},{20,40}},
                  rotation=0)));
          Engine.Components.TimingBelt timing_belt annotation (Placement(
                transformation(extent={{-100,-60},{-60,-20}}, rotation=0)));
          Engine.Components.Cam intake_cam(vo=ivo, vc=ivc) annotation (
                                Placement(transformation(extent={{-60,60},{
                    -40,80}}, rotation=0)));
          Engine.Components.Cam exhaust_cam(vo=evo, vc=evc) annotation (
                               Placement(transformation(extent={{60,60},{40,
                    80}}, rotation=0)));
          Engine.Components.SparkControl spark_control(spark_advance=spark_advance) 
              annotation (                          Placement(transformation(
                  extent={{-24,44},{-4,64}}, rotation=0)));
          Engine.Components.ChamberVolume chamber_volume annotation (
                            Placement(transformation(extent={{56,-20},{76,0}},
                  rotation=0)));
          OffsetShaft offset_shaft(shift=crank_shift) annotation (
                                Placement(transformation(extent={{-20,-140},{
                    20,-180}}, rotation=0)));
          HeatTransfer_Hohenberg heatTransfer 
                                    annotation (Placement(transformation(extent=
                   {{-48,-34},{-28,-14}})));
          Combustion_Weibe_Hohenberg_Biogas combustion_Weibe_Hohenberg_Biogas 
            annotation (Placement(transformation(extent={{-10,18},{10,38}})));
          Interfaces.BiogasPropertyRequired bioP annotation (Placement(
                transformation(
                extent={{-10,-10},{10,10}},
                rotation=180,
                origin={110,8})));
        equation
          connect(crankslider.piston, chamber_volume.piston) annotation (Line(
              points={{0,-80},{66,-80},{66,-18}},
              color={0,127,0},
              smooth=Smooth.None));
          connect(crankslider.crank, offset_shaft.cyl) annotation (Line(
              points={{0,-108},{0,-140}},
              color={0,0,0},
              smooth=Smooth.None));
          connect(crankslider.crank, timing_belt.crankshaft) annotation (Line(
              points={{0,-108},{-80,-108},{-80,-56}},
              color={0,0,0},
              smooth=Smooth.None));
          connect(crankslider.crank, spark_control.crank) annotation (Line(
              points={{0,-108},{-14,-108},{-14,44}},
              color={0,0,0},
              smooth=Smooth.None));
          connect(offset_shaft.crank, crankshaft) annotation (Line(
              points={{-20,-160},{-20,-198},{0,-198}},
              color={0,0,0},
              smooth=Smooth.None));
          connect(timing_belt.camshaft, exhaust_cam.camshaft) annotation (Line(
              points={{-80,-28},{-80,86},{60,86},{60,70},{52.8,70}},
              color={0,0,0},
              smooth=Smooth.None));
          connect(timing_belt.camshaft, intake_cam.camshaft) annotation (Line(
              points={{-80,-28},{-80,70},{-52.8,70}},
              color={0,0,0},
              smooth=Smooth.None));
          connect(intake_valve.a, intake) annotation (Line(
              points={{-40,30},{-100,30},{-100,80}},
              color={255,0,0},
              smooth=Smooth.None));
          connect(exhaust_valve.a, exhaust) annotation (Line(
              points={{40,30},{100,30},{100,80}},
              color={255,0,0},
              smooth=Smooth.None));
          connect(intake_cam.valve_lift, intake_valve.lift) annotation (Line(
              points={{-40,70},{-40,62.5},{-20,62.5},{-20,55},{-30,55},{-30,
                  40}},
              color={0,127,0},
              smooth=Smooth.None));
          connect(exhaust_cam.valve_lift, exhaust_valve.lift) annotation (Line(
              points={{40,70},{40,62.5},{24,62.5},{24,55},{30,55},{30,40}},
              color={0,127,0},
              smooth=Smooth.None));
          connect(combustion_chamber.state, piston.chamber) annotation (Line(
              points={{0,-10},{0,-30}},
              color={255,0,0},
              smooth=Smooth.None));
          connect(piston.piston, crankslider.piston) annotation (Line(
              points={{0,-42.2},{0,-80}},
              color={0,127,0},
              smooth=Smooth.None));
          connect(chamber_volume.volume, combustion_chamber.volume) annotation (
             Line(
              points={{55,-10},{11,-10}},
              color={0,0,127},
              smooth=Smooth.None));
          connect(combustion_chamber.state, exhaust_valve.b) annotation (Line(
              points={{0,-10},{6,-10},{6,12},{30,12},{30,20}},
              color={255,0,0},
              smooth=Smooth.None));
          connect(combustion_chamber.state, intake_valve.b) annotation (Line(
              points={{0,-10},{-12,-10},{-12,12},{-30,12},{-30,20}},
              color={255,0,0},
              smooth=Smooth.None));
          connect(geom, chamber_volume.geom) annotation (Line(
              points={{110,-50},{80,-50},{80,-10},{77,-10}},
              color={0,0,0},
              smooth=Smooth.None));
          connect(geom, piston.geom) annotation (Line(
              points={{110,-50},{80,-50},{80,-40},{11,-40}},
              color={0,0,0},
              smooth=Smooth.None));
          connect(geom, crankslider.geom) annotation (Line(
              points={{110,-50},{80,-50},{80,-100},{22,-100}},
              color={0,0,0},
              smooth=Smooth.None));
          connect(crankslider.piston, heatTransfer.piston) annotation (Line(
              points={{0,-80},{-37.8,-80},{-37.8,-14}},
              color={0,127,0},
              smooth=Smooth.None));
          connect(heatTransfer.geom, geom) annotation (Line(
              points={{-27,-24},{80,-24},{80,-50},{110,-50}},
              color={0,0,0},
              smooth=Smooth.None));
          connect(chamber_volume.volume, heatTransfer.volume) annotation (Line(
              points={{55,-10},{40,-10},{40,-19},{-27.4,-19}},
              color={0,0,127},
              smooth=Smooth.None));
          connect(combustion_chamber.state, heatTransfer.state) annotation (
              Line(
              points={{0,-10},{-38,-10},{-38,-24}},
              color={255,0,0},
              smooth=Smooth.None));
          connect(spark_control.spark, combustion_Weibe_Hohenberg_Biogas.start) 
            annotation (Line(
              points={{-3,54},{0,54},{0,39}},
              color={255,0,255},
              smooth=Smooth.None));
          connect(combustion_Weibe_Hohenberg_Biogas.cylinder,
            combustion_chamber.state) annotation (Line(
              points={{0,24},{0,-10}},
              color={255,0,0},
              smooth=Smooth.None));
          connect(combustion_Weibe_Hohenberg_Biogas.crank, crankslider.crank) 
            annotation (Line(
              points={{-10,24},{-22,24},{-22,-108},{0,-108}},
              color={0,0,0},
              smooth=Smooth.None));
          connect(combustion_Weibe_Hohenberg_Biogas.mass, combustion_chamber.mass) 
            annotation (Line(
              points={{-6,17},{-6,1}},
              color={0,0,127},
              smooth=Smooth.None));
          connect(heatTransfer.qloss, combustion_Weibe_Hohenberg_Biogas.qloss) 
            annotation (Line(
              points={{-38,-13},{-38,8},{8,8},{8,17}},
              color={0,0,127},
              smooth=Smooth.None));
          connect(bioP, combustion_Weibe_Hohenberg_Biogas.bioP) annotation (
              Line(
              points={{110,8},{68,8},{68,10},{26,10},{26,27.4},{11,27.4}},
              color={0,0,0},
              smooth=Smooth.None));
          annotation (
            Window(
              x=0.07,
              y=0.03,
              width=0.99,
              height=0.91),
            Icon(coordinateSystem(
                preserveAspectRatio=false,
                extent={{-100,-200},{100,100}},
                grid={2,2}), graphics={
                Polygon(
                  points={{-60,-140},{-60,40},{-42,40},{-42,44},{-40,46},{-60,
                      60},{-100,60},{-100,-140},{-60,-140}},
                  lineColor={0,0,0},
                  fillColor={192,192,192},
                  fillPattern=FillPattern.Solid),
                Polygon(
                  points={{-100,96},{-60,96},{-40,80},{-20,60},{-20,46},{-18,
                      44},{-18,40},{18,40},{18,44},{20,46},{20,60},{40,80},{
                      60,96},{100,96},{100,100},{-100,100},{-100,96}},
                  lineColor={0,0,0},
                  fillColor={192,192,192},
                  fillPattern=FillPattern.Solid),
                Rectangle(
                  extent={{-58,12},{58,-64}},
                  lineColor={0,0,0},
                  fillPattern=FillPattern.VerticalCylinder,
                  fillColor={192,192,192}),
                Rectangle(
                  extent={{-60,6},{60,0}},
                  lineColor={0,0,0},
                  fillColor={0,0,0},
                  fillPattern=FillPattern.Solid),
                Rectangle(
                  extent={{-60,-8},{60,-14}},
                  lineColor={0,0,0},
                  fillColor={0,0,0},
                  fillPattern=FillPattern.Solid),
                Rectangle(
                  extent={{-60,-20},{60,-26}},
                  lineColor={0,0,0},
                  fillColor={0,0,0},
                  fillPattern=FillPattern.Solid),
                Polygon(
                  points={{-58,-64},{-40,-52},{40,-52},{58,-64},{-58,-64}},
                  lineColor={0,0,255},
                  pattern=LinePattern.None,
                  fillColor={255,255,255},
                  fillPattern=FillPattern.Solid),
                Ellipse(
                  extent={{-4,-34},{4,-42}},
                  lineColor={0,0,0},
                  fillColor={0,0,0},
                  fillPattern=FillPattern.Solid),
                Ellipse(extent={{-40,-240},{40,-160}}, lineColor={192,192,192}),
                Line(
                  points={{0,-200},{30,-174},{0,-38}},
                  color={0,0,0},
                  thickness=1),
                Rectangle(
                  extent={{-42,36},{-18,32}},
                  lineColor={0,0,0},
                  fillPattern=FillPattern.VerticalCylinder,
                  fillColor={192,192,192}),
                Polygon(
                  points={{-42,36},{-40,38},{-20,38},{-18,36},{-42,36}},
                  lineColor={160,160,164},
                  fillColor={192,192,192},
                  fillPattern=FillPattern.Solid),
                Rectangle(
                  extent={{-32,38},{-28,96}},
                  lineColor={0,0,0},
                  fillPattern=FillPattern.VerticalCylinder,
                  fillColor={192,192,192}),
                Polygon(
                  points={{18,44},{20,46},{40,46},{42,44},{18,44}},
                  lineColor={160,160,164},
                  fillColor={192,192,192},
                  fillPattern=FillPattern.Solid),
                Rectangle(
                  extent={{18,44},{42,40}},
                  lineColor={0,0,0},
                  fillPattern=FillPattern.VerticalCylinder,
                  fillColor={192,192,192}),
                Rectangle(
                  extent={{28,46},{32,96}},
                  lineColor={0,0,0},
                  fillPattern=FillPattern.VerticalCylinder,
                  fillColor={192,192,192}),
                Polygon(
                  points={{100,-140},{100,60},{60,60},{40,46},{42,44},{42,40},
                      {60,40},{60,-140},{100,-140}},
                  lineColor={0,0,0},
                  fillColor={192,192,192},
                  fillPattern=FillPattern.Solid),
                Text(extent={{-60,-100},{60,-120}}, textString=
                                                        "%name")}),
            Documentation(info="All the basic componets of a single cylinder, 2-valve engine
have been compiled in this model.
"),         Diagram(coordinateSystem(preserveAspectRatio=true, extent={{-100,
                    -200},{100,100}},
                grid={2,2}),           graphics));
        end IndividualCylinder_Weibe_Hohenberg_Biogas;

        model I4_Engine_Biogas "An Inline 4 Cylinder Engine"
          extends Interfaces.Engine;
          parameter Types.Degrees spark_advance "Spark advance";
          parameter Types.Degrees burn_duration "Burn Duration";
          parameter Types.Degrees evo=40 "Exhaust Valve Opening";
          parameter Types.Degrees ivo=150 "Intake Valve Opening";
          parameter Types.Degrees evc=205 "Exhaust Valve Closing";
          parameter Types.Degrees ivc=310 "Intake Valve Closing";
          parameter Modelica.SIunits.Diameter ivd=0.032 "Intake Valve Diameter";
          parameter Modelica.SIunits.Diameter evd=0.028
            "Exhaust Valve Diameter";
          Modelica.SIunits.Power power "Instantaneous engine power output";
          replaceable model CylinderType = IndividualCylinder constrainedby
            Interfaces.Cylinder;
          Modelica.Mechanics.Rotational.Components.Inertia crankshaft_inertia(
                                                                   J=0.03, w(start=
                 157)) annotation (Placement(transformation(
                origin={-80,-50},
                extent={{-10,-10},{10,10}},
                rotation=90)));
          IndividualCylinder_Weibe_Hohenberg_Biogas
            individualCylinder_Weibe_Hohenberg_Biogas(
            spark_advance=spark_advance,
            burn_duration=burn_duration,
            evo=evo,
            ivo=ivo,
            evc=evc,
            ivc=ivc,
            crank_shift=0,
            ivd=ivd,
            evd=evd) annotation (Placement(transformation(extent={{-100,-20},{
                    -60,40}})));
          IndividualCylinder_Weibe_Hohenberg_Biogas
            individualCylinder_Weibe_Hohenberg_Biogas1(
            spark_advance=spark_advance,
            burn_duration=burn_duration,
            evo=evo,
            ivo=ivo,
            evc=evc,
            ivc=ivc,
            ivd=ivd,
            evd=evd,
            crank_shift=540) 
            annotation (Placement(transformation(extent={{-52,-20},{-12,40}})));
          IndividualCylinder_Weibe_Hohenberg_Biogas
            individualCylinder_Weibe_Hohenberg_Biogas2(
            spark_advance=spark_advance,
            burn_duration=burn_duration,
            evo=evo,
            ivo=ivo,
            evc=evc,
            ivc=ivc,
            ivd=ivd,
            evd=evd,
            crank_shift=360) 
            annotation (Placement(transformation(extent={{0,-20},{40,40}})));
          IndividualCylinder_Weibe_Hohenberg_Biogas
            individualCylinder_Weibe_Hohenberg_Biogas3(
            spark_advance=spark_advance,
            burn_duration=burn_duration,
            evo=evo,
            ivo=ivo,
            evc=evc,
            ivc=ivc,
            ivd=ivd,
            evd=evd,
            crank_shift=180) 
            annotation (Placement(transformation(extent={{48,-20},{88,40}})));
          Interfaces.BiogasPropertyRequired bioP 
            annotation (Placement(transformation(extent={{98,-56},{118,-36}})));
        equation
          connect(crankshaft_inertia.flange_a, crankshaft) annotation (Line(
                points={{-80,-60},{-80,-70},{-100,-70},{-100,-40}}, color={0,
                  0,0}));
          power = crankshaft.tau*der(crankshaft.phi);
          connect(individualCylinder_Weibe_Hohenberg_Biogas.crankshaft,
            individualCylinder_Weibe_Hohenberg_Biogas1.crankshaft) annotation (
              Line(
              points={{-80,-19.6},{-32,-19.6}},
              color={0,0,0},
              smooth=Smooth.None));
          connect(individualCylinder_Weibe_Hohenberg_Biogas1.crankshaft,
            individualCylinder_Weibe_Hohenberg_Biogas2.crankshaft) annotation (
              Line(
              points={{-32,-19.6},{20,-19.6}},
              color={0,0,0},
              smooth=Smooth.None));
          connect(individualCylinder_Weibe_Hohenberg_Biogas2.crankshaft,
            individualCylinder_Weibe_Hohenberg_Biogas3.crankshaft) annotation (
              Line(
              points={{20,-19.6},{68,-19.6}},
              color={0,0,0},
              smooth=Smooth.None));
          connect(crankshaft_inertia.flange_b,
            individualCylinder_Weibe_Hohenberg_Biogas.crankshaft) annotation (
              Line(
              points={{-80,-40},{-80,-19.6}},
              color={0,0,0},
              smooth=Smooth.None));
          connect(bioP, individualCylinder_Weibe_Hohenberg_Biogas3.bioP) 
            annotation (Line(
              points={{108,-46},{92,-46},{92,21.6},{90,21.6}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(bioP, individualCylinder_Weibe_Hohenberg_Biogas1.bioP) 
            annotation (Line(
              points={{108,-46},{-4,-46},{-4,21.6},{-10,21.6}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(individualCylinder_Weibe_Hohenberg_Biogas2.bioP, bioP) 
            annotation (Line(
              points={{42,21.6},{46,21.6},{46,-46},{108,-46}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(individualCylinder_Weibe_Hohenberg_Biogas.bioP, bioP) 
            annotation (Line(
              points={{-58,21.6},{-56,21.6},{-56,22},{-54,22},{-54,-46},{108,
                  -46}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(individualCylinder_Weibe_Hohenberg_Biogas3.geom,
            engine_geometry) annotation (Line(
              points={{90,10},{96,10},{96,0},{110,0}},
              color={0,0,0},
              smooth=Smooth.None));
          connect(individualCylinder_Weibe_Hohenberg_Biogas2.geom,
            engine_geometry) annotation (Line(
              points={{42,10},{50,10},{50,-32},{96,-32},{96,0},{110,0}},
              color={0,0,0},
              smooth=Smooth.None));
          connect(individualCylinder_Weibe_Hohenberg_Biogas1.geom,
            engine_geometry) annotation (Line(
              points={{-10,10},{-6,10},{-6,-32},{96,-32},{96,0},{110,0}},
              color={0,0,0},
              smooth=Smooth.None));
          connect(individualCylinder_Weibe_Hohenberg_Biogas.geom,
            engine_geometry) annotation (Line(
              points={{-58,10},{-50,10},{-50,-32},{96,-32},{96,0},{110,0}},
              color={0,0,0},
              smooth=Smooth.None));
          connect(individualCylinder_Weibe_Hohenberg_Biogas.intake, intake) 
            annotation (Line(
              points={{-100,36},{-100,80}},
              color={255,0,0},
              smooth=Smooth.None));
          connect(individualCylinder_Weibe_Hohenberg_Biogas1.intake, intake) 
            annotation (Line(
              points={{-52,36},{-52,60},{-100,60},{-100,80}},
              color={255,0,0},
              smooth=Smooth.None));
          connect(individualCylinder_Weibe_Hohenberg_Biogas2.intake, intake) 
            annotation (Line(
              points={{0,36},{0,60},{-100,60},{-100,80}},
              color={255,0,0},
              smooth=Smooth.None));
          connect(individualCylinder_Weibe_Hohenberg_Biogas3.intake, intake) 
            annotation (Line(
              points={{48,36},{48,60},{-100,60},{-100,80}},
              color={255,0,0},
              smooth=Smooth.None));
          connect(individualCylinder_Weibe_Hohenberg_Biogas3.exhaust, exhaust) 
            annotation (Line(
              points={{88,36},{100,36},{100,80}},
              color={255,0,0},
              smooth=Smooth.None));
          connect(individualCylinder_Weibe_Hohenberg_Biogas2.exhaust, exhaust) 
            annotation (Line(
              points={{40,36},{40,52},{100,52},{100,80}},
              color={255,0,0},
              smooth=Smooth.None));
          connect(individualCylinder_Weibe_Hohenberg_Biogas1.exhaust, exhaust) 
            annotation (Line(
              points={{-12,36},{-12,52},{100,52},{100,80}},
              color={255,0,0},
              smooth=Smooth.None));
          connect(individualCylinder_Weibe_Hohenberg_Biogas.exhaust, exhaust) 
            annotation (Line(
              points={{-60,36},{-60,52},{100,52},{100,80}},
              color={255,0,0},
              smooth=Smooth.None));
          annotation (
            Icon(
              coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},
                    {100,100}},
                grid={2,2}),
              graphics={
                Polygon(
                  points={{-48,32},{-56,-8},{-56,-48},{-46,-76},{-12,-76},{0,
                      -48},{0,-8},{-8,32},{-46,32},{-48,32}},
                  lineColor={0,0,0},
                  fillColor={160,160,164},
                  fillPattern=FillPattern.Solid),
                Ellipse(
                  extent={{-46,-10},{-16,-68}},
                  lineColor={0,0,0},
                  fillColor={192,192,192},
                  fillPattern=FillPattern.Solid),
                Ellipse(
                  extent={{-36,-30},{-26,-50}},
                  lineColor={0,0,0},
                  fillPattern=FillPattern.HorizontalCylinder,
                  fillColor={192,192,192}),
                Rectangle(
                  extent={{-90,-30},{-30,-50}},
                  lineColor={0,0,0},
                  fillPattern=FillPattern.HorizontalCylinder,
                  fillColor={192,192,192}),
                Polygon(
                  points={{-8,32},{72,32},{84,-6},{84,-48},{74,-76},{-12,-76},
                      {0,-48},{0,-8},{-8,32},{-8,32}},
                  lineColor={0,0,0},
                  fillColor={192,192,192},
                  fillPattern=FillPattern.Solid),
                Line(points={{0,-6},{84,-6}}, color={160,160,164}),
                Line(points={{0,-48},{84,-48}}, color={0,0,0}),
                Rectangle(
                  extent={{-24,48},{-34,32}},
                  lineColor={0,0,0},
                  fillColor={128,128,128},
                  fillPattern=FillPattern.Solid),
                Rectangle(
                  extent={{72,48},{62,32}},
                  lineColor={0,0,0},
                  fillColor={128,128,128},
                  fillPattern=FillPattern.Solid),
                Ellipse(
                  extent={{60,70},{74,40}},
                  lineColor={0,0,0},
                  fillColor={192,192,192},
                  fillPattern=FillPattern.Solid),
                Line(points={{-36,58},{-46,-30}}, color={0,0,0}),
                Rectangle(
                  extent={{-28,62},{68,48}},
                  lineColor={0,0,0},
                  fillPattern=FillPattern.HorizontalCylinder,
                  fillColor={192,192,192}),
                Ellipse(
                  extent={{-36,70},{-24,44}},
                  lineColor={0,0,0},
                  fillColor={192,192,192},
                  fillPattern=FillPattern.Solid),
                Line(points={{-24,58},{-16,-36}}, color={0,0,0}),
                Rectangle(
                  extent={{-16,64},{-10,42}},
                  lineColor={160,160,164},
                  fillColor={160,160,164},
                  fillPattern=FillPattern.Solid),
                Rectangle(
                  extent={{-8,68},{-2,46}},
                  lineColor={160,160,164},
                  fillColor={160,160,164},
                  fillPattern=FillPattern.Solid),
                Rectangle(
                  extent={{4,66},{10,44}},
                  lineColor={160,160,164},
                  fillColor={160,160,164},
                  fillPattern=FillPattern.Solid),
                Rectangle(
                  extent={{12,64},{18,42}},
                  lineColor={160,160,164},
                  fillColor={160,160,164},
                  fillPattern=FillPattern.Solid),
                Rectangle(
                  extent={{26,68},{32,46}},
                  lineColor={160,160,164},
                  fillColor={160,160,164},
                  fillPattern=FillPattern.Solid),
                Rectangle(
                  extent={{34,66},{40,44}},
                  lineColor={160,160,164},
                  fillColor={160,160,164},
                  fillPattern=FillPattern.Solid),
                Rectangle(
                  extent={{46,64},{52,42}},
                  lineColor={160,160,164},
                  fillColor={160,160,164},
                  fillPattern=FillPattern.Solid),
                Rectangle(
                  extent={{54,66},{60,44}},
                  lineColor={160,160,164},
                  fillColor={160,160,164},
                  fillPattern=FillPattern.Solid),
                Rectangle(
                  extent={{-16,42},{-10,32}},
                  lineColor={0,0,0},
                  fillColor={192,192,192},
                  fillPattern=FillPattern.Forward),
                Rectangle(
                  extent={{-8,44},{-2,32}},
                  lineColor={0,0,0},
                  fillColor={192,192,192},
                  fillPattern=FillPattern.Forward),
                Rectangle(
                  extent={{4,44},{10,32}},
                  lineColor={0,0,0},
                  fillColor={192,192,192},
                  fillPattern=FillPattern.Forward),
                Rectangle(
                  extent={{12,42},{18,32}},
                  lineColor={0,0,0},
                  fillColor={192,192,192},
                  fillPattern=FillPattern.Forward),
                Rectangle(
                  extent={{26,44},{32,32}},
                  lineColor={0,0,0},
                  fillColor={192,192,192},
                  fillPattern=FillPattern.Forward),
                Rectangle(
                  extent={{34,44},{40,32}},
                  lineColor={0,0,0},
                  fillColor={192,192,192},
                  fillPattern=FillPattern.Forward),
                Rectangle(
                  extent={{46,42},{52,32}},
                  lineColor={0,0,0},
                  fillColor={192,192,192},
                  fillPattern=FillPattern.Forward),
                Rectangle(
                  extent={{54,44},{60,32}},
                  lineColor={0,0,0},
                  fillColor={192,192,192},
                  fillPattern=FillPattern.Forward),
                Line(
                  points={{-100,80},{-48,32}},
                  color={255,127,0},
                  thickness=0.5),
                Line(points={{100,80},{62,32}}, color={255,127,0}),
                Text(extent={{-60,-74},{80,-106}}, textString=
                                                       "%name")}),
            Window(
              x=0.39,
              y=0.34,
              width=0.6,
              height=0.6),
            Documentation(info="An assembly of components required for an I4 engine.  This model requires a connection to intake and exhaust
gases, the crankshaft and geometry information.
"),         Diagram(coordinateSystem(preserveAspectRatio=false,extent={{-100,
                    -100},{100,100}},
                grid={2,2}),           graphics));
        end I4_Engine_Biogas;

        model HeatTransfer_Eichelberg
          "Heat Transfer of the cylinder by using Eichelberg's equation"
          // Modelica.SIunits.Volume Vc=volume "Chamber volume";

          // Modelica.SIunits.Temperature Tg=state.T "Gas Temperature";

        Modelica.SIunits.Temp_C Tw = 160 "Cylinder Wall Temperature";

        // Real hloss "Coefficient Of Heat Transfer";
        // Real Cm "Pistion Velocity";
        // Real Aloss "Heat transfer Area";
          // Modelica.SIunits.Heat qloss "Heat transfer";

        protected
          Modelica.SIunits.CoefficientOfHeatTransfer hloss
            "Coefficient Of Heat Transfer";
          Modelica.SIunits.Velocity Cm "Pistion Velocity";
          Modelica.SIunits.Area Aloss "Heat transfer Area";

        public
          Modelica.Mechanics.Translational.Interfaces.Flange_a piston annotation (
                                         Placement(transformation(extent={{-10,90},
                    {10,110}}, rotation=0)));
          Interfaces.EngineGeometryRequired geom annotation (Placement(
                transformation(
                origin={110,0},
                extent={{-10,-10},{10,10}},
                rotation=180)));
          Modelica.Blocks.Interfaces.RealOutput qloss annotation (Placement(
                transformation(
                extent={{-10,-10},{10,10}},
                rotation=90,
                origin={0,110})));

        public
          Interfaces.Gas state "Gas state" annotation (
              layer="icon",
            Placement(transformation(extent={{-10,-10},{10,10}}, rotation=0)));
        equation
          Cm=abs(der(piston.s));
          hloss=7.8*Cm^0.33333*(state.P/1000000*(state.T+273.15))^0.5;
          Aloss=Modelica.Constants.pi*geom.bore*(geom.bore/2+piston.s);
          qloss=hloss*Aloss*(state.T-Tw);
          piston.f=0;
          state.mdot=0;
          state.q=0;

          annotation (Icon(coordinateSystem(preserveAspectRatio=true, extent={{-100,
                    -100},{100,100}}), graphics),
                                      Diagram(coordinateSystem(
                  preserveAspectRatio=false, extent={{-100,-100},{100,100}}),
                graphics));
        end HeatTransfer_Eichelberg;

        model IndividualCylinder_Weibe_Eichelberg_Biogas
          "Collection of parts for a complete individual cylinder"
          extends Interfaces.Cylinder;
          parameter Types.Degrees spark_advance "Spark advance";
          parameter Types.Degrees burn_duration "Burn Duration";
          parameter Types.Degrees evo=40 "Exhaust Valve Opening";
          parameter Types.Degrees ivo=150 "Intake Valve Opening";
          parameter Types.Degrees evc=205 "Exhaust Valve Closing";
          parameter Types.Degrees ivc=310 "Intake Valve Closing";
          parameter Types.Degrees crank_shift=0 "Crankshaft Shift";
          parameter Modelica.SIunits.Diameter ivd=0.032 "Intake Valve Diameter";
          parameter Modelica.SIunits.Diameter evd=0.028
            "Exhaust Valve Diameter";
          Engine.Components.MasslessPiston piston annotation (
                        Placement(transformation(extent={{-10,-50},{10,-30}},
                  rotation=0)));
          Engine.Components.CrankSlider crankslider annotation (
                            Placement(transformation(extent={{-20,-120},{20,
                    -80}}, rotation=0)));
          Engine.Components.ControlVolume combustion_chamber annotation (Placement(
                transformation(extent={{-10,-20},{10,0}}, rotation=0)));
          Engine.Components.Valve intake_valve(dia=ivd) annotation (
                              Placement(transformation(extent={{-40,20},{-20,
                    40}}, rotation=0)));
          Engine.Components.Valve exhaust_valve(dia=evd) annotation (
                             Placement(transformation(extent={{40,20},{20,40}},
                  rotation=0)));
          Engine.Components.TimingBelt timing_belt annotation (Placement(
                transformation(extent={{-100,-60},{-60,-20}}, rotation=0)));
          Engine.Components.Cam intake_cam(vo=ivo, vc=ivc) annotation (
                                Placement(transformation(extent={{-60,60},{
                    -40,80}}, rotation=0)));
          Engine.Components.Cam exhaust_cam(vo=evo, vc=evc) annotation (
                               Placement(transformation(extent={{60,60},{40,
                    80}}, rotation=0)));
          Engine.Components.SparkControl spark_control(spark_advance=spark_advance) 
              annotation (                          Placement(transformation(
                  extent={{-24,44},{-4,64}}, rotation=0)));
          Engine.Components.ChamberVolume chamber_volume annotation (
                            Placement(transformation(extent={{56,-20},{76,0}},
                  rotation=0)));
          OffsetShaft offset_shaft(shift=crank_shift) annotation (
                                Placement(transformation(extent={{-20,-140},{
                    20,-180}}, rotation=0)));
          Combustion_Weibe_Hohenberg_Biogas combustion_Weibe_Hohenberg_Biogas 
            annotation (Placement(transformation(extent={{-10,18},{10,38}})));
          Interfaces.BiogasPropertyRequired bioP annotation (Placement(
                transformation(
                extent={{-10,-10},{10,10}},
                rotation=180,
                origin={110,8})));
          HeatTransfer_Eichelberg heatTransfer_Eichelberg 
            annotation (Placement(transformation(extent={{-52,-18},{-32,2}})));
        equation
          connect(crankslider.piston, chamber_volume.piston) annotation (Line(
              points={{0,-80},{66,-80},{66,-18}},
              color={0,127,0},
              smooth=Smooth.None));
          connect(crankslider.crank, offset_shaft.cyl) annotation (Line(
              points={{0,-108},{0,-140}},
              color={0,0,0},
              smooth=Smooth.None));
          connect(crankslider.crank, timing_belt.crankshaft) annotation (Line(
              points={{0,-108},{-80,-108},{-80,-56}},
              color={0,0,0},
              smooth=Smooth.None));
          connect(crankslider.crank, spark_control.crank) annotation (Line(
              points={{0,-108},{-14,-108},{-14,44}},
              color={0,0,0},
              smooth=Smooth.None));
          connect(offset_shaft.crank, crankshaft) annotation (Line(
              points={{-20,-160},{-20,-198},{0,-198}},
              color={0,0,0},
              smooth=Smooth.None));
          connect(timing_belt.camshaft, exhaust_cam.camshaft) annotation (Line(
              points={{-80,-28},{-80,86},{60,86},{60,70},{52.8,70}},
              color={0,0,0},
              smooth=Smooth.None));
          connect(timing_belt.camshaft, intake_cam.camshaft) annotation (Line(
              points={{-80,-28},{-80,70},{-52.8,70}},
              color={0,0,0},
              smooth=Smooth.None));
          connect(intake_valve.a, intake) annotation (Line(
              points={{-40,30},{-100,30},{-100,80}},
              color={255,0,0},
              smooth=Smooth.None));
          connect(exhaust_valve.a, exhaust) annotation (Line(
              points={{40,30},{100,30},{100,80}},
              color={255,0,0},
              smooth=Smooth.None));
          connect(intake_cam.valve_lift, intake_valve.lift) annotation (Line(
              points={{-40,70},{-40,62.5},{-20,62.5},{-20,55},{-30,55},{-30,
                  40}},
              color={0,127,0},
              smooth=Smooth.None));
          connect(exhaust_cam.valve_lift, exhaust_valve.lift) annotation (Line(
              points={{40,70},{40,62.5},{24,62.5},{24,55},{30,55},{30,40}},
              color={0,127,0},
              smooth=Smooth.None));
          connect(combustion_chamber.state, piston.chamber) annotation (Line(
              points={{0,-10},{0,-30}},
              color={255,0,0},
              smooth=Smooth.None));
          connect(piston.piston, crankslider.piston) annotation (Line(
              points={{0,-42.2},{0,-80}},
              color={0,127,0},
              smooth=Smooth.None));
          connect(chamber_volume.volume, combustion_chamber.volume) annotation (
             Line(
              points={{55,-10},{11,-10}},
              color={0,0,127},
              smooth=Smooth.None));
          connect(combustion_chamber.state, exhaust_valve.b) annotation (Line(
              points={{0,-10},{6,-10},{6,12},{30,12},{30,20}},
              color={255,0,0},
              smooth=Smooth.None));
          connect(combustion_chamber.state, intake_valve.b) annotation (Line(
              points={{0,-10},{-12,-10},{-12,12},{-30,12},{-30,20}},
              color={255,0,0},
              smooth=Smooth.None));
          connect(geom, chamber_volume.geom) annotation (Line(
              points={{110,-50},{80,-50},{80,-10},{77,-10}},
              color={0,0,0},
              smooth=Smooth.None));
          connect(geom, piston.geom) annotation (Line(
              points={{110,-50},{80,-50},{80,-40},{11,-40}},
              color={0,0,0},
              smooth=Smooth.None));
          connect(geom, crankslider.geom) annotation (Line(
              points={{110,-50},{80,-50},{80,-100},{22,-100}},
              color={0,0,0},
              smooth=Smooth.None));
          connect(spark_control.spark, combustion_Weibe_Hohenberg_Biogas.start) 
            annotation (Line(
              points={{-3,54},{0,54},{0,39}},
              color={255,0,255},
              smooth=Smooth.None));
          connect(combustion_Weibe_Hohenberg_Biogas.cylinder,
            combustion_chamber.state) annotation (Line(
              points={{0,24},{0,-10}},
              color={255,0,0},
              smooth=Smooth.None));
          connect(combustion_Weibe_Hohenberg_Biogas.crank, crankslider.crank) 
            annotation (Line(
              points={{-10,24},{-22,24},{-22,-108},{0,-108}},
              color={0,0,0},
              smooth=Smooth.None));
          connect(combustion_Weibe_Hohenberg_Biogas.mass, combustion_chamber.mass) 
            annotation (Line(
              points={{-6,17},{-6,1}},
              color={0,0,127},
              smooth=Smooth.None));
          connect(bioP, combustion_Weibe_Hohenberg_Biogas.bioP) annotation (
              Line(
              points={{110,8},{68,8},{68,10},{26,10},{26,27.4},{11,27.4}},
              color={0,0,0},
              smooth=Smooth.None));
          connect(crankslider.piston, heatTransfer_Eichelberg.piston) 
            annotation (Line(
              points={{0,-80},{-42,-80},{-42,2}},
              color={0,127,0},
              smooth=Smooth.None));
          connect(heatTransfer_Eichelberg.state, combustion_chamber.state) 
            annotation (Line(
              points={{-42,-8},{0,-8},{0,-10}},
              color={255,0,0},
              smooth=Smooth.None));
          connect(heatTransfer_Eichelberg.qloss,
            combustion_Weibe_Hohenberg_Biogas.qloss) annotation (Line(
              points={{-42,3},{-42,10},{8,10},{8,17}},
              color={0,0,127},
              smooth=Smooth.None));
          connect(heatTransfer_Eichelberg.geom, geom) annotation (Line(
              points={{-31,-8},{-28,-8},{-28,4},{80,4},{80,-50},{110,-50}},
              color={0,0,0},
              smooth=Smooth.None));
          annotation (
            Window(
              x=0.07,
              y=0.03,
              width=0.99,
              height=0.91),
            Icon(coordinateSystem(
                preserveAspectRatio=false,
                extent={{-100,-200},{100,100}},
                grid={2,2}), graphics={
                Polygon(
                  points={{-60,-140},{-60,40},{-42,40},{-42,44},{-40,46},{-60,
                      60},{-100,60},{-100,-140},{-60,-140}},
                  lineColor={0,0,0},
                  fillColor={192,192,192},
                  fillPattern=FillPattern.Solid),
                Polygon(
                  points={{-100,96},{-60,96},{-40,80},{-20,60},{-20,46},{-18,
                      44},{-18,40},{18,40},{18,44},{20,46},{20,60},{40,80},{
                      60,96},{100,96},{100,100},{-100,100},{-100,96}},
                  lineColor={0,0,0},
                  fillColor={192,192,192},
                  fillPattern=FillPattern.Solid),
                Rectangle(
                  extent={{-58,12},{58,-64}},
                  lineColor={0,0,0},
                  fillPattern=FillPattern.VerticalCylinder,
                  fillColor={192,192,192}),
                Rectangle(
                  extent={{-60,6},{60,0}},
                  lineColor={0,0,0},
                  fillColor={0,0,0},
                  fillPattern=FillPattern.Solid),
                Rectangle(
                  extent={{-60,-8},{60,-14}},
                  lineColor={0,0,0},
                  fillColor={0,0,0},
                  fillPattern=FillPattern.Solid),
                Rectangle(
                  extent={{-60,-20},{60,-26}},
                  lineColor={0,0,0},
                  fillColor={0,0,0},
                  fillPattern=FillPattern.Solid),
                Polygon(
                  points={{-58,-64},{-40,-52},{40,-52},{58,-64},{-58,-64}},
                  lineColor={0,0,255},
                  pattern=LinePattern.None,
                  fillColor={255,255,255},
                  fillPattern=FillPattern.Solid),
                Ellipse(
                  extent={{-4,-34},{4,-42}},
                  lineColor={0,0,0},
                  fillColor={0,0,0},
                  fillPattern=FillPattern.Solid),
                Ellipse(extent={{-40,-240},{40,-160}}, lineColor={192,192,192}),
                Line(
                  points={{0,-200},{30,-174},{0,-38}},
                  color={0,0,0},
                  thickness=1),
                Rectangle(
                  extent={{-42,36},{-18,32}},
                  lineColor={0,0,0},
                  fillPattern=FillPattern.VerticalCylinder,
                  fillColor={192,192,192}),
                Polygon(
                  points={{-42,36},{-40,38},{-20,38},{-18,36},{-42,36}},
                  lineColor={160,160,164},
                  fillColor={192,192,192},
                  fillPattern=FillPattern.Solid),
                Rectangle(
                  extent={{-32,38},{-28,96}},
                  lineColor={0,0,0},
                  fillPattern=FillPattern.VerticalCylinder,
                  fillColor={192,192,192}),
                Polygon(
                  points={{18,44},{20,46},{40,46},{42,44},{18,44}},
                  lineColor={160,160,164},
                  fillColor={192,192,192},
                  fillPattern=FillPattern.Solid),
                Rectangle(
                  extent={{18,44},{42,40}},
                  lineColor={0,0,0},
                  fillPattern=FillPattern.VerticalCylinder,
                  fillColor={192,192,192}),
                Rectangle(
                  extent={{28,46},{32,96}},
                  lineColor={0,0,0},
                  fillPattern=FillPattern.VerticalCylinder,
                  fillColor={192,192,192}),
                Polygon(
                  points={{100,-140},{100,60},{60,60},{40,46},{42,44},{42,40},
                      {60,40},{60,-140},{100,-140}},
                  lineColor={0,0,0},
                  fillColor={192,192,192},
                  fillPattern=FillPattern.Solid),
                Text(extent={{-60,-100},{60,-120}}, textString=
                                                        "%name")}),
            Documentation(info="All the basic componets of a single cylinder, 2-valve engine
have been compiled in this model.
"),         Diagram(coordinateSystem(preserveAspectRatio=false,extent={{-100,
                    -200},{100,100}},
                grid={2,2}),           graphics));
        end IndividualCylinder_Weibe_Eichelberg_Biogas;

        model I4_Engine_Biogas_Eichelberg "An Inline 4 Cylinder Engine"
          extends Interfaces.Engine;
          parameter Types.Degrees spark_advance "Spark advance";
          parameter Types.Degrees burn_duration "Burn Duration";
          parameter Types.Degrees evo=40 "Exhaust Valve Opening";
          parameter Types.Degrees ivo=150 "Intake Valve Opening";
          parameter Types.Degrees evc=205 "Exhaust Valve Closing";
          parameter Types.Degrees ivc=310 "Intake Valve Closing";
          parameter Modelica.SIunits.Diameter ivd=0.032 "Intake Valve Diameter";
          parameter Modelica.SIunits.Diameter evd=0.028
            "Exhaust Valve Diameter";
          Modelica.SIunits.Power power "Instantaneous engine power output";
          replaceable model CylinderType = IndividualCylinder constrainedby
            Interfaces.Cylinder;
          Modelica.Mechanics.Rotational.Components.Inertia crankshaft_inertia(
                                                                   J=0.03, w(start=
                 157)) annotation (Placement(transformation(
                origin={-80,-50},
                extent={{-10,-10},{10,10}},
                rotation=90)));
          Interfaces.BiogasPropertyRequired bioP 
            annotation (Placement(transformation(extent={{98,-56},{118,-36}})));
          IndividualCylinder_Weibe_Eichelberg_Biogas
            individualCylinder_Weibe_Eichelberg_Biogas(spark_advance=20,
              burn_duration=60) annotation (Placement(transformation(extent={{
                    -100,-20},{-60,40}})));
          IndividualCylinder_Weibe_Eichelberg_Biogas
            individualCylinder_Weibe_Eichelberg_Biogas1(
            spark_advance=20,
            burn_duration=60,
            crank_shift=540) 
            annotation (Placement(transformation(extent={{-50,-20},{-10,40}})));
          IndividualCylinder_Weibe_Eichelberg_Biogas
            individualCylinder_Weibe_Eichelberg_Biogas2(
            spark_advance=20,
            burn_duration=60,
            crank_shift=360) 
            annotation (Placement(transformation(extent={{0,-20},{40,40}})));
          IndividualCylinder_Weibe_Eichelberg_Biogas
            individualCylinder_Weibe_Eichelberg_Biogas3(
            spark_advance=20,
            burn_duration=60,
            crank_shift=180) 
            annotation (Placement(transformation(extent={{50,-20},{90,40}})));
        equation
          connect(crankshaft_inertia.flange_a, crankshaft) annotation (Line(
                points={{-80,-60},{-80,-70},{-100,-70},{-100,-40}}, color={0,
                  0,0}));
          power = crankshaft.tau*der(crankshaft.phi);
          connect(individualCylinder_Weibe_Eichelberg_Biogas.crankshaft,
            individualCylinder_Weibe_Eichelberg_Biogas1.crankshaft) annotation (
             Line(
              points={{-80,-19.6},{-30,-19.6}},
              color={0,0,0},
              smooth=Smooth.None));
          connect(individualCylinder_Weibe_Eichelberg_Biogas1.crankshaft,
            individualCylinder_Weibe_Eichelberg_Biogas2.crankshaft) annotation (
             Line(
              points={{-30,-19.6},{20,-19.6}},
              color={0,0,0},
              smooth=Smooth.None));
          connect(individualCylinder_Weibe_Eichelberg_Biogas2.crankshaft,
            individualCylinder_Weibe_Eichelberg_Biogas3.crankshaft) annotation (
             Line(
              points={{20,-19.6},{70,-19.6}},
              color={0,0,0},
              smooth=Smooth.None));
          connect(intake, individualCylinder_Weibe_Eichelberg_Biogas.intake) 
            annotation (Line(
              points={{-100,80},{-100,80},{-100,36}},
              color={255,0,0},
              smooth=Smooth.None));
          connect(intake, individualCylinder_Weibe_Eichelberg_Biogas1.intake) 
            annotation (Line(
              points={{-100,80},{-100,56},{-50,56},{-50,36}},
              color={255,0,0},
              smooth=Smooth.None));
          connect(intake, individualCylinder_Weibe_Eichelberg_Biogas2.intake) 
            annotation (Line(
              points={{-100,80},{-100,56},{0,56},{0,36}},
              color={255,0,0},
              smooth=Smooth.None));
          connect(intake, individualCylinder_Weibe_Eichelberg_Biogas3.intake) 
            annotation (Line(
              points={{-100,80},{-100,56},{50,56},{50,36}},
              color={255,0,0},
              smooth=Smooth.None));
          connect(exhaust, individualCylinder_Weibe_Eichelberg_Biogas3.exhaust) 
            annotation (Line(
              points={{100,80},{100,36},{90,36}},
              color={255,0,0},
              smooth=Smooth.None));
          connect(exhaust, individualCylinder_Weibe_Eichelberg_Biogas2.exhaust) 
            annotation (Line(
              points={{100,80},{100,46},{40,46},{40,36}},
              color={255,0,0},
              smooth=Smooth.None));
          connect(exhaust, individualCylinder_Weibe_Eichelberg_Biogas1.exhaust) 
            annotation (Line(
              points={{100,80},{100,46},{-10,46},{-10,36}},
              color={255,0,0},
              smooth=Smooth.None));
          connect(exhaust, individualCylinder_Weibe_Eichelberg_Biogas.exhaust) 
            annotation (Line(
              points={{100,80},{100,46},{-60,46},{-60,36}},
              color={255,0,0},
              smooth=Smooth.None));
          connect(engine_geometry, individualCylinder_Weibe_Eichelberg_Biogas3.geom) 
            annotation (Line(
              points={{110,0},{98,0},{98,10},{92,10}},
              color={0,0,0},
              smooth=Smooth.None));
          connect(engine_geometry, individualCylinder_Weibe_Eichelberg_Biogas2.geom) 
            annotation (Line(
              points={{110,0},{98,0},{98,-32},{46,-32},{46,10},{42,10}},
              color={0,0,0},
              smooth=Smooth.None));
          connect(engine_geometry, individualCylinder_Weibe_Eichelberg_Biogas1.geom) 
            annotation (Line(
              points={{110,0},{98,0},{98,-32},{-2,-32},{-2,10},{-8,10}},
              color={0,0,0},
              smooth=Smooth.None));
          connect(engine_geometry, individualCylinder_Weibe_Eichelberg_Biogas.geom) 
            annotation (Line(
              points={{110,0},{98,0},{98,-32},{-52,-32},{-52,10},{-58,10}},
              color={0,0,0},
              smooth=Smooth.None));
          connect(individualCylinder_Weibe_Eichelberg_Biogas3.bioP, bioP) 
            annotation (Line(
              points={{92,21.6},{94,21.6},{94,-46},{108,-46}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(individualCylinder_Weibe_Eichelberg_Biogas2.bioP, bioP) 
            annotation (Line(
              points={{42,21.6},{48,21.6},{48,-46},{108,-46}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(individualCylinder_Weibe_Eichelberg_Biogas1.bioP, bioP) 
            annotation (Line(
              points={{-8,21.6},{-4,21.6},{-4,-46},{108,-46}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(individualCylinder_Weibe_Eichelberg_Biogas.bioP, bioP) 
            annotation (Line(
              points={{-58,21.6},{-56,21.6},{-56,-46},{108,-46}},
              color={0,0,255},
              smooth=Smooth.None));
          connect(individualCylinder_Weibe_Eichelberg_Biogas.crankshaft,
            crankshaft_inertia.flange_b) annotation (Line(
              points={{-80,-19.6},{-80,-40}},
              color={0,0,0},
              smooth=Smooth.None));
          annotation (
            Icon(
              coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},
                    {100,100}},
                grid={2,2}),
              graphics={
                Polygon(
                  points={{-48,32},{-56,-8},{-56,-48},{-46,-76},{-12,-76},{0,
                      -48},{0,-8},{-8,32},{-46,32},{-48,32}},
                  lineColor={0,0,0},
                  fillColor={160,160,164},
                  fillPattern=FillPattern.Solid),
                Ellipse(
                  extent={{-46,-10},{-16,-68}},
                  lineColor={0,0,0},
                  fillColor={192,192,192},
                  fillPattern=FillPattern.Solid),
                Ellipse(
                  extent={{-36,-30},{-26,-50}},
                  lineColor={0,0,0},
                  fillPattern=FillPattern.HorizontalCylinder,
                  fillColor={192,192,192}),
                Rectangle(
                  extent={{-90,-30},{-30,-50}},
                  lineColor={0,0,0},
                  fillPattern=FillPattern.HorizontalCylinder,
                  fillColor={192,192,192}),
                Polygon(
                  points={{-8,32},{72,32},{84,-6},{84,-48},{74,-76},{-12,-76},
                      {0,-48},{0,-8},{-8,32},{-8,32}},
                  lineColor={0,0,0},
                  fillColor={192,192,192},
                  fillPattern=FillPattern.Solid),
                Line(points={{0,-6},{84,-6}}, color={160,160,164}),
                Line(points={{0,-48},{84,-48}}, color={0,0,0}),
                Rectangle(
                  extent={{-24,48},{-34,32}},
                  lineColor={0,0,0},
                  fillColor={128,128,128},
                  fillPattern=FillPattern.Solid),
                Rectangle(
                  extent={{72,48},{62,32}},
                  lineColor={0,0,0},
                  fillColor={128,128,128},
                  fillPattern=FillPattern.Solid),
                Ellipse(
                  extent={{60,70},{74,40}},
                  lineColor={0,0,0},
                  fillColor={192,192,192},
                  fillPattern=FillPattern.Solid),
                Line(points={{-36,58},{-46,-30}}, color={0,0,0}),
                Rectangle(
                  extent={{-28,62},{68,48}},
                  lineColor={0,0,0},
                  fillPattern=FillPattern.HorizontalCylinder,
                  fillColor={192,192,192}),
                Ellipse(
                  extent={{-36,70},{-24,44}},
                  lineColor={0,0,0},
                  fillColor={192,192,192},
                  fillPattern=FillPattern.Solid),
                Line(points={{-24,58},{-16,-36}}, color={0,0,0}),
                Rectangle(
                  extent={{-16,64},{-10,42}},
                  lineColor={160,160,164},
                  fillColor={160,160,164},
                  fillPattern=FillPattern.Solid),
                Rectangle(
                  extent={{-8,68},{-2,46}},
                  lineColor={160,160,164},
                  fillColor={160,160,164},
                  fillPattern=FillPattern.Solid),
                Rectangle(
                  extent={{4,66},{10,44}},
                  lineColor={160,160,164},
                  fillColor={160,160,164},
                  fillPattern=FillPattern.Solid),
                Rectangle(
                  extent={{12,64},{18,42}},
                  lineColor={160,160,164},
                  fillColor={160,160,164},
                  fillPattern=FillPattern.Solid),
                Rectangle(
                  extent={{26,68},{32,46}},
                  lineColor={160,160,164},
                  fillColor={160,160,164},
                  fillPattern=FillPattern.Solid),
                Rectangle(
                  extent={{34,66},{40,44}},
                  lineColor={160,160,164},
                  fillColor={160,160,164},
                  fillPattern=FillPattern.Solid),
                Rectangle(
                  extent={{46,64},{52,42}},
                  lineColor={160,160,164},
                  fillColor={160,160,164},
                  fillPattern=FillPattern.Solid),
                Rectangle(
                  extent={{54,66},{60,44}},
                  lineColor={160,160,164},
                  fillColor={160,160,164},
                  fillPattern=FillPattern.Solid),
                Rectangle(
                  extent={{-16,42},{-10,32}},
                  lineColor={0,0,0},
                  fillColor={192,192,192},
                  fillPattern=FillPattern.Forward),
                Rectangle(
                  extent={{-8,44},{-2,32}},
                  lineColor={0,0,0},
                  fillColor={192,192,192},
                  fillPattern=FillPattern.Forward),
                Rectangle(
                  extent={{4,44},{10,32}},
                  lineColor={0,0,0},
                  fillColor={192,192,192},
                  fillPattern=FillPattern.Forward),
                Rectangle(
                  extent={{12,42},{18,32}},
                  lineColor={0,0,0},
                  fillColor={192,192,192},
                  fillPattern=FillPattern.Forward),
                Rectangle(
                  extent={{26,44},{32,32}},
                  lineColor={0,0,0},
                  fillColor={192,192,192},
                  fillPattern=FillPattern.Forward),
                Rectangle(
                  extent={{34,44},{40,32}},
                  lineColor={0,0,0},
                  fillColor={192,192,192},
                  fillPattern=FillPattern.Forward),
                Rectangle(
                  extent={{46,42},{52,32}},
                  lineColor={0,0,0},
                  fillColor={192,192,192},
                  fillPattern=FillPattern.Forward),
                Rectangle(
                  extent={{54,44},{60,32}},
                  lineColor={0,0,0},
                  fillColor={192,192,192},
                  fillPattern=FillPattern.Forward),
                Line(
                  points={{-100,80},{-48,32}},
                  color={255,127,0},
                  thickness=0.5),
                Line(points={{100,80},{62,32}}, color={255,127,0}),
                Text(extent={{-60,-74},{80,-106}}, textString=
                                                       "%name")}),
            Window(
              x=0.39,
              y=0.34,
              width=0.6,
              height=0.6),
            Documentation(info="An assembly of components required for an I4 engine.  This model requires a connection to intake and exhaust
gases, the crankshaft and geometry information.
"),         Diagram(coordinateSystem(preserveAspectRatio=false,extent={{-100,
                    -100},{100,100}},
                grid={2,2}),           graphics));
        end I4_Engine_Biogas_Eichelberg;

        model ExhaustManifold
          import SI = Modelica.SIunits;
          parameter Integer n=1;
        parameter SI.Volume V=1;
        parameter SI.Length D=0.1;
        parameter SI.Area A= Modelica.Constants.pi*D^2/4;
         //SI.Density ro(start=1.293);
        //EditionGasEngine.BioDES2.Engine.Engine.GasProperties.SimpleAirProperties stat(P=P,T=T);
        SI.Mass m;
        SI.Temperature T(start=300);
        SI.Pressure P;

          Interfaces.Gas gas[n] 
            annotation (Placement(transformation(extent={{-80,-100},{-60,-80}})));

          Interfaces.Gas exhust 
            annotation (Placement(transformation(extent={{80,-20},{100,0}})));

          GasProperties.SimpleGasProperty stat(P=P, T=T) 
            annotation (Placement(transformation(extent={{-74,42},{-54,62}})));

        initial equation
          //der(T)=0;
          T=300;
          P=101325;
          //m=2;
        equation
          der(T)*stat.cv*m=sum(gas.q)+stat.h*exhust.mdot-stat.cv*T*der(m);
          der(m)=sum(gas.mdot)+exhust.mdot;
          //sum(gas.mdot)+exhust.mdot=0;
          P*V=m*T*Modelica.Constants.R/stat.mw;
          //m=stat.ro *V;
          exhust.mdot=-A*0.61*sqrt(abs((P-exhust.P)*stat.ro*2));
          //ro=1.293*(P/101325)*(273.15/T);
          //exhust.mdot=gas1.mdot+gas2.mdot;
          //exhust.P=stat.P;
         // exhust.T=stat.T;
         // exhust.q=exhust.mdot*stat.h;
          annotation (Icon(graphics={Rectangle(
                  extent={{-100,60},{80,-80}},
                  lineColor={0,0,255},
                  fillColor={255,255,0},
                  fillPattern=FillPattern.Solid)}), Diagram(graphics));
        end ExhaustManifold;
        annotation (             Window(
            x=0.26,
            y=0.11,
            width=0.4,
            height=0.4,
            library=1,
            autolayout=1));
      end Components;

      model testSoruce
      parameter Modelica.SIunits.Pressure P=101325 "Gas pressure";
      parameter Modelica.SIunits.Temperature T= 300 "Gas temperature";
      parameter Modelica.SIunits.MassFlowRate mdot "Mass flow rate";
      //parameter Modelica.SIunits.HeatFlowRate q "Heat flow rate";

        Interfaces.Gas gas 
          annotation (Placement(transformation(extent={{80,0},{100,20}})));
        GasProperties.SimpleGasProperty stat(P=P,T=T) 
          annotation (Placement(transformation(extent={{-82,24},{-62,44}})));
      equation
        gas.P=P;
        gas.T=T;
        gas.mdot=-mdot;
       gas.q=-mdot*stat.h;
      end testSoruce;

      model Sink
      parameter Modelica.SIunits.MassFlowRate mdot=1;
      parameter Modelica.SIunits.Pressure P=101325 "Gas pressure";
      parameter Modelica.SIunits.Temperature T= 300 "Gas temperature";
        Interfaces.Gas gas 
          annotation (Placement(transformation(extent={{80,0},{100,20}})));
      equation
        //gas.mdot=mdot;
        gas.P=P;
        gas.T=T;
      end Sink;
      annotation (
        Window(
          x=0.42,
          y=0.12,
          width=0.54,
          height=0.8,
          library=1,
          autolayout=1),
        Documentation(info="This package contains numerous engine component models and a complete I4 engine model.
"),     Icon(coordinateSystem(
            preserveAspectRatio=false,
            extent={{0,0},{842,836}},
            grid={1,1}), graphics={
            Ellipse(
              extent={{-84,-22},{-38,-42}},
              lineColor={0,0,0},
              fillColor={192,192,192},
              fillPattern=FillPattern.Solid),
            Ellipse(
              extent={{-84,-20},{-38,-40}},
              lineColor={0,0,0},
              fillPattern=FillPattern.Sphere,
              fillColor={192,192,192}),
            Rectangle(
              extent={{-64,22},{-59,-30}},
              lineColor={0,0,0},
              fillPattern=FillPattern.VerticalCylinder,
              fillColor={192,192,192}),
            Ellipse(
              extent={{19,-79},{25,-85}},
              lineColor={0,0,0},
              fillColor={0,0,0},
              fillPattern=FillPattern.Solid),
            Line(points={{22,-82},{36,-71},{21,-39}}, color={0,0,0}),
            Rectangle(
              extent={{3,-23},{37,-39}},
              lineColor={0,0,0},
              fillPattern=FillPattern.VerticalCylinder,
              fillColor={192,192,192})}));
    end Engine;

    package TestSimulation "A set of Test Simulation of Bio Engine package"
      extends Modelica.Icons.Example;

      model Race "Race a car from 0-100 kilometers per hour"
        extends Modelica.Icons.Example;
        Vehicles.SportsCar sports_car annotation (
            Placement(transformation(extent={{-60,-40},{40,60}}, rotation=0)));
        Chassis.Road race_track annotation (Placement(transformation(extent={
                  {-50,-100},{-10,-60}}, rotation=0)));
      equation
        connect(race_track.road_surface, sports_car.road) annotation (Line(
              points={{-30,-60},{-30,-40}}, color={0,255,0}));
        when (sports_car.speed.signal[1] > 100) then
          terminate("Simulation from 0-100 kilometers per hour completed");
        end when;
        annotation (
          Window(
            x=0.32,
            y=0.13,
            width=0.6,
            height=0.6),
          Diagram(coordinateSystem(
              preserveAspectRatio=false,
              extent={{-100,-100},{100,100}},
              grid={2,2}), graphics),
          Documentation(info="This example allows you to study the effects of various vehicle/transmission/engine parameters.
The simulation will automatically stop once the vehicle has reached 100 kilometers per hour unless a shorter
simulation time is specified.
"));
      end Race;

      model CylinderOnDyno "Single cylinder engine connected to a dynamometer"
        extends Modelica.Icons.Example;
        Engine.Components.Reservoir intake(P=50000) annotation (Placement(
              transformation(extent={{-60,60},{-40,80}}, rotation=0)));
        Engine.Components.Reservoir exhaust annotation (Placement(
              transformation(extent={{40,60},{60,80}}, rotation=0)));
        Engine.Components.IndividualCylinder cylinder(
          spark_advance=20,
          burn_duration=60,
          combustion_chamber,
          evo=40,
          ivo=150,
          evc=205,
          ivc=310)            annotation (Placement(transformation(extent={{
                  -20,-20},{20,40}}, rotation=0)));
        Engine.Components.Dynamometer dyno annotation (Placement(
              transformation(extent={{-40,-70},{-20,-50}}, rotation=0)));
        Modelica.Blocks.Sources.Ramp speed_profile(
          height=2000,
          duration=1,
          offset=1500,
          startTime=1)   annotation (Placement(transformation(extent={{-80,
                  -70},{-60,-50}}, rotation=0)));
        Engine.GeometrySource sample_geometry(
          bore=.08,
          stroke=.08,
          conrod=.152,
          comp_ratio=9.5) annotation (                        Placement(
              transformation(extent={{60,0},{80,20}}, rotation=0)));
      equation
        connect(dyno.shaft, cylinder.crankshaft) annotation (Line(
            points={{-20,-60},{0,-60},{0,-19.6}},
            color={0,0,0},
            smooth=Smooth.None));
        connect(sample_geometry.geom, cylinder.geom) annotation (Line(
            points={{59,10},{22,10}},
            color={0,0,0},
            smooth=Smooth.None));
        connect(cylinder.intake, intake.tap) annotation (Line(
            points={{-20,36},{-50,36},{-50,60}},
            color={255,0,0},
            smooth=Smooth.None));
        connect(speed_profile.y, dyno.rpm) annotation (Line(
            points={{-59,-60},{-41,-60}},
            color={0,0,127},
            smooth=Smooth.None));
        connect(cylinder.exhaust, exhaust.tap) annotation (Line(
            points={{20,36},{36,36},{36,50},{50,50},{50,60}},
            color={255,0,0},
            smooth=Smooth.None));
        annotation (
          Window(
            x=0.1,
            y=0.21,
            width=0.6,
            height=0.6),
          Documentation(info="This simulation involves a single cylinder connected
to a dynamometer.  One interesting thing to look at
in this simulation is the instantaneous torque felt
by the dynamometer vs. the average torque.  Both are
available as variables to be viewed within the dynamometer
model.
"),       Diagram(coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},
                  {100,100}},
              grid={2,2}),     graphics));
      end CylinderOnDyno;

      model CylinderAndInertia "Single cylinder engine connected to a flywheel"
        extends Modelica.Icons.Example;
        Engine.Components.Reservoir intake(P=50000) annotation (Placement(
              transformation(extent={{-60,60},{-40,80}}, rotation=0)));
        Engine.Components.Reservoir exhaust annotation (Placement(
              transformation(extent={{40,60},{60,80}}, rotation=0)));
        Engine.Components.IndividualCylinder cylinder1(spark_advance=20,
            burn_duration=60) annotation (Placement(transformation(extent={{
                  -20,-20},{20,40}}, rotation=0)));
        Engine.GeometrySource sample_geometry(
          bore=.08,
          stroke=.08,
          conrod=.152,
          comp_ratio=9.5) annotation (Placement(transformation(extent={{60,0},
                  {80,20}}, rotation=0)));
        Modelica.Mechanics.Rotational.Components.Inertia flywheel(
                                                       J=10) annotation (Placement(
              transformation(extent={{-40,-70},{-20,-50}}, rotation=0)));
        Modelica.Mechanics.Rotational.Sources.Torque starter 
                                                     annotation (Placement(
              transformation(extent={{-68,-70},{-48,-50}}, rotation=0)));
        Modelica.Blocks.Sources.Step starter_torque(height=100, startTime=1) 
          annotation (Placement(transformation(extent={{-100,-70},{-80,-50}},
                rotation=0)));
      equation
        connect(starter_torque.y, starter.tau) annotation (Line(
            points={{-79,-60},{-70,-60}},
            color={0,0,127},
            smooth=Smooth.None));
        connect(starter.flange, flywheel.flange_a) annotation (Line(
            points={{-48,-60},{-40,-60}},
            color={0,0,0},
            smooth=Smooth.None));
        connect(flywheel.flange_b, cylinder1.crankshaft) annotation (Line(
            points={{-20,-60},{0,-60},{0,-19.6}},
            color={0,0,0},
            smooth=Smooth.None));
        connect(sample_geometry.geom, cylinder1.geom) annotation (Line(
            points={{59,10},{22,10}},
            color={0,0,0},
            smooth=Smooth.None));
        connect(cylinder1.exhaust, exhaust.tap) annotation (Line(
            points={{20,36},{50,36},{50,60}},
            color={255,0,0},
            smooth=Smooth.None));
        connect(cylinder1.intake, intake.tap) annotation (Line(
            points={{-20,36},{-50,36},{-50,60}},
            color={255,0,0},
            smooth=Smooth.None));
        annotation (
          Diagram(coordinateSystem(preserveAspectRatio=false,extent={{-100,
                  -100},{100,100}},
              grid={2,2}),     graphics),
          Window(
            x=0.32,
            y=0.13,
            width=0.6,
            height=0.6),
          Documentation(info="This is a simulation of a single cylinder engine connected to a
flywheel.  The idea is to see the acceleration of the flywheel.
This is in contrast to other models where dynamometers are used
to keep the speed fixed.
"));
      end CylinderAndInertia;

      model I4EngineOnDyno "I4 engine connected to a dynamometer"
        extends Modelica.Icons.Example;
        Engine.Components.Dynamometer dyno annotation (Placement(
              transformation(extent={{-100,-76},{-60,-36}}, rotation=0)));
        Engine.Components.Reservoir intake(P=101800) annotation (
                         Placement(transformation(extent={{-60,60},{-20,100}},
                rotation=0)));
        Engine.Components.Reservoir exhaust annotation (Placement(
              transformation(extent={{20,20},{60,60}}, rotation=0)));
        Engine.SportsCarGeometry sample_geometry(
          bore=.08,
          stroke=.08,
          conrod=.152,
          comp_ratio=9.5) annotation (                            Placement(
              transformation(extent={{60,-60},{100,-20}}, rotation=0)));
        Engine.Components.I4_Engine I4(
          spark_advance=20,
          burn_duration=60,
          evo=40) annotation (Placement(transformation(extent={{-40,-80},{40,
                  0}}, rotation=0)));
        Engine.Components.Manifold intake_manifold annotation (Placement(
              transformation(extent={{-60,8},{-20,48}}, rotation=0)));
        Modelica.Blocks.Sources.Constant throttle_angle(k=90)   annotation (
                                   Placement(transformation(extent={{-140,18},
                  {-120,38}}, rotation=0)));
        Modelica.Blocks.Sources.Constant rpm(k=1500)            annotation (
                                   Placement(transformation(extent={{-140,18},
                  {-120,38}}, rotation=0)));
        Engine.BiogasPropertySource biogasPropertySource(
          lhv=44700000,
          afr=14.6,
          SGF=1) 
          annotation (Placement(transformation(extent={{70,-76},{90,-56}})));
      equation
        connect(dyno.shaft, I4.crankshaft) annotation (Line(
            points={{-60,-56},{-40,-56}},
            color={0,0,0},
            smooth=Smooth.None));
        connect(I4.engine_geometry, sample_geometry.geom) annotation (Line(
            points={{44,-40},{58,-40}},
            color={0,0,0},
            smooth=Smooth.None));
        connect(I4.exhaust, exhaust.tap) annotation (Line(
            points={{40,-8},{40,20}},
            color={255,0,0},
            smooth=Smooth.None));
        connect(I4.intake, intake_manifold.manifold) annotation (Line(
            points={{-40,-8},{-40,8}},
            color={255,0,0},
            smooth=Smooth.None));
        connect(intake_manifold.ambient, intake.tap) annotation (Line(
            points={{-40,48},{-40,60}},
            color={255,0,0},
            smooth=Smooth.None));
        connect(throttle_angle.y, intake_manifold.throttle_angle) annotation (
            Line(
            points={{-119,28},{-62,28}},
            color={0,0,127},
            smooth=Smooth.None));
        connect(rpm.y, dyno.rpm) annotation (Line(
            points={{-119,28},{-110.5,28},{-110.5,-56},{-102,-56}},
            color={0,0,127},
            smooth=Smooth.None));
        connect(I4.biogasPropertyRequired, biogasPropertySource.bioP) 
          annotation (Line(
            points={{43.2,-58.4},{56,-58.4},{56,-67},{69,-67}},
            color={0,0,255},
            smooth=Smooth.None));
        annotation (
          Window(
            x=0.32,
            y=0.07,
            width=0.6,
            height=0.6),
          Documentation(info="This is a simulation of an I4 engine connected to a dynamometer.  The interesting
thing about this simulation that sets it apart from the single cylinder case is
that this model will demonstrate manifold filling and emptying effects.
"),       Diagram(coordinateSystem(preserveAspectRatio=false,extent={{-150,
                  -100},{100,100}},
              grid={2,2}),     graphics));
      end I4EngineOnDyno;

      model I4EngineAndInertia "I4 engine connected to a flywheel"
        extends Modelica.Icons.Example;
        Engine.Components.I4_Engine engine(spark_advance=20, burn_duration=60) 
          annotation (                           Placement(transformation(
                extent={{40,-36},{60,-16}}, rotation=0)));
        Engine.GeometrySource geometry(
          bore=0.080,
          stroke=0.080,
          conrod=0.157) annotation (                            Placement(
              transformation(extent={{80,-36},{100,-16}}, rotation=0)));
        Engine.Components.Reservoir intake_manifold annotation (
                     Placement(transformation(extent={{20,0},{40,20}},
                rotation=0)));
        Engine.Components.Reservoir exhaust_manifold annotation (
                     Placement(transformation(extent={{60,0},{80,20}},
                rotation=0)));
        Modelica.Mechanics.Rotational.Components.Inertia flywheel(
                                                       J=10, w(start=157)) 
          annotation (                          Placement(transformation(extent={{0,-40},
                  {20,-20}}, rotation=0)));
      equation
        connect(flywheel.flange_b, engine.crankshaft) annotation (Line(
            points={{20,-30},{-32,-30},{-32,-30},{40,-30}},
            color={0,0,0},
            smooth=Smooth.None));
        connect(geometry.geom, engine.engine_geometry) annotation (Line(
            points={{79,-26},{38,-26},{38,-26},{61,-26}},
            color={0,0,0},
            smooth=Smooth.None));
        connect(intake_manifold.tap, engine.intake) annotation (Line(
            points={{30,0},{30,-18},{40,-18}},
            color={255,0,0},
            smooth=Smooth.None));
        connect(engine.exhaust, exhaust_manifold.tap) annotation (Line(
            points={{60,-18},{70,-18},{70,0}},
            color={255,0,0},
            smooth=Smooth.None));
        annotation (
          Window(
            x=0.27,
            y=0.1,
            width=0.6,
            height=0.6),
          Documentation(info="This model simulates an I4 engine accelerating an inertia.
"),       Diagram(coordinateSystem(preserveAspectRatio=false,extent={{-100,
                  -100},{100,100}},
              grid={2,2}),     graphics));
      end I4EngineAndInertia;

      model CylinderOnDyno_Weibe_Hohenburg
        "Single cylinder engine connected to a dynamometer"
        extends Modelica.Icons.Example;
        Engine.Components.Reservoir intake(P=50000) annotation (Placement(
              transformation(extent={{-60,60},{-40,80}}, rotation=0)));
        Engine.Components.Reservoir exhaust annotation (Placement(
              transformation(extent={{40,60},{60,80}}, rotation=0)));
        Engine.Components.Dynamometer dyno annotation (
            Placement(transformation(extent={{-40,-70},{-20,-50}}, rotation=0)));
        Modelica.Blocks.Sources.Ramp speed_profile(
          height=2000,
          duration=1,
          offset=1500,
          startTime=1)   annotation (                             Placement(
              transformation(extent={{-80,-70},{-60,-50}}, rotation=0)));
        Engine.GeometrySource sample_geometry(
          bore=.08,
          stroke=.08,
          conrod=.152,
          comp_ratio=9.5) annotation (                        Placement(
              transformation(extent={{60,0},{80,20}}, rotation=0)));
        Engine.Components.IndividualCylinder_Weibe_Hohenberg
          individualCylinder_Weibe_Hohenberg(spark_advance=20, burn_duration=60) 
          annotation (Placement(transformation(extent={{-22,-22},{24,46}})));
      equation
        connect(speed_profile.y, dyno.rpm) annotation (Line(
            points={{-59,-60},{-54.5,-60},{-54.5,-80},{-50,-80},{-50,-40},{
                -41,-40}},
            color={0,0,127},
            smooth=Smooth.None));
        connect(intake.tap, individualCylinder_Weibe_Hohenberg.intake) 
          annotation (Line(
            points={{-50,60},{-50,41.4667},{-22,41.4667}},
            color={255,0,0},
            smooth=Smooth.None));
        connect(individualCylinder_Weibe_Hohenberg.exhaust, exhaust.tap) 
          annotation (Line(
            points={{24,41.4667},{50,41.4667},{50,60}},
            color={255,0,0},
            smooth=Smooth.None));
        connect(individualCylinder_Weibe_Hohenberg.crankshaft, dyno.shaft) 
          annotation (Line(
            points={{1,-21.5467},{1,-60},{-20,-60}},
            color={0,0,0},
            smooth=Smooth.None));
        connect(individualCylinder_Weibe_Hohenberg.geom, sample_geometry.geom) 
          annotation (Line(
            points={{26.3,12},{44.65,12},{44.65,10},{59,10}},
            color={0,0,0},
            smooth=Smooth.None));
        annotation (
          Window(
            x=0.1,
            y=0.21,
            width=0.6,
            height=0.6),
          Documentation(info="This simulation involves a single cylinder connected
to a dynamometer.  One interesting thing to look at
in this simulation is the instantaneous torque felt
by the dynamometer vs. the average torque.  Both are
available as variables to be viewed within the dynamometer
model.
"),       Diagram(coordinateSystem(preserveAspectRatio=false,extent={{-100,
                  -100},{100,100}},
              grid={2,2}),     graphics));
      end CylinderOnDyno_Weibe_Hohenburg;

      model CylinderOnDyno_Weibe
        "Single cylinder engine connected to a dynamometer"
        extends Modelica.Icons.Example;
        Engine.Components.Reservoir intake(P=100000) 
                                                    annotation (Placement(
              transformation(extent={{-60,60},{-40,80}}, rotation=0)));
        Engine.Components.Reservoir exhaust annotation (Placement(
              transformation(extent={{40,60},{60,80}}, rotation=0)));
        Engine.Components.Dynamometer dyno annotation (Placement(
              transformation(extent={{-40,-70},{-20,-50}}, rotation=0)));
        Modelica.Blocks.Sources.Ramp speed_profile(
          height=2000,
          duration=1,
          offset=1500,
          startTime=10)  annotation (Placement(transformation(extent={{-80,
                  -70},{-60,-50}}, rotation=0)));
        Engine.GeometrySource sample_geometry(
          bore=.08,
          stroke=.08,
          conrod=.152,
          comp_ratio=9.5) annotation (                        Placement(
              transformation(extent={{60,0},{80,20}}, rotation=0)));
        Engine.Components.IndividualCylinder_Weibe individualCylinder_Weibe(
            spark_advance=20, burn_duration=60) 
          annotation (Placement(transformation(extent={{-20,-22},{20,38}})));
      equation
        connect(speed_profile.y, dyno.rpm) annotation (Line(
            points={{-59,-60},{-41,-60}},
            color={0,0,127},
            smooth=Smooth.None));
        connect(intake.tap, individualCylinder_Weibe.intake) annotation (Line(
            points={{-50,60},{-50,34},{-20,34}},
            color={255,0,0},
            smooth=Smooth.None));
        connect(individualCylinder_Weibe.exhaust, exhaust.tap) annotation (Line(
            points={{20,34},{50,34},{50,60}},
            color={255,0,0},
            smooth=Smooth.None));
        connect(dyno.shaft, individualCylinder_Weibe.crankshaft) annotation (
            Line(
            points={{-20,-60},{0,-60},{0,-21.6}},
            color={0,0,0},
            smooth=Smooth.None));
        connect(individualCylinder_Weibe.geom, sample_geometry.geom) 
          annotation (Line(
            points={{22,8},{40.5,8},{40.5,10},{59,10}},
            color={0,0,0},
            smooth=Smooth.None));
        annotation (
          Window(
            x=0.1,
            y=0.21,
            width=0.6,
            height=0.6),
          Documentation(info="This simulation involves a single cylinder connected
to a dynamometer.  One interesting thing to look at
in this simulation is the instantaneous torque felt
by the dynamometer vs. the average torque.  Both are
available as variables to be viewed within the dynamometer
model.
"),       Diagram(coordinateSystem(preserveAspectRatio=false,extent={{-100,
                  -100},{100,100}},
              grid={2,2}),     graphics));
      end CylinderOnDyno_Weibe;

      model CylinderOnDyno_Weibe_Hohenburg_Biogas
        "Single cylinder engine connected to a dynamometer"
        extends Modelica.Icons.Example;
        Engine.Components.Reservoir intake(P=50000) annotation (Placement(
              transformation(extent={{-60,60},{-40,80}}, rotation=0)));
        Engine.Components.Reservoir exhaust annotation (
            Placement(transformation(extent={{40,60},{60,80}}, rotation=0)));
        Engine.Components.Dynamometer dyno annotation (
            Placement(transformation(extent={{-40,-70},{-20,-50}}, rotation=0)));
        Modelica.Blocks.Sources.Ramp speed_profile(
          height=2000,
          duration=1,
          startTime=1,
          offset=1500)   annotation (                             Placement(
              transformation(extent={{-80,-70},{-60,-50}}, rotation=0)));
        Engine.GeometrySource sample_geometry(
          bore=.08,
          stroke=.08,
          conrod=.152,
          comp_ratio=9.5) annotation (                        Placement(
              transformation(extent={{60,0},{80,20}}, rotation=0)));
        Engine.Components.IndividualCylinder_Weibe_Hohenberg_Biogas
          individualCylinder_Weibe_Hohenberg_Biogas(spark_advance=20,
            burn_duration=60) 
          annotation (Placement(transformation(extent={{-16,-12},{22,46}})));
        Engine.BiogasPropertySource biogasPropertySource(
          lhv=44600000,
          afr=14.6,
          SGF=1) 
          annotation (Placement(transformation(extent={{64,28},{84,48}})));
      equation
        connect(speed_profile.y, dyno.rpm) annotation (Line(
            points={{-59,-60},{-54.5,-60},{-54.5,-80},{-50,-80},{-50,-40},{
                -41,-40}},
            color={0,0,127},
            smooth=Smooth.None));
        connect(sample_geometry.geom, individualCylinder_Weibe_Hohenberg_Biogas.geom) 
          annotation (Line(
            points={{59,10},{32,10},{32,17},{23.9,17}},
            color={0,0,0},
            smooth=Smooth.None));
        connect(dyno.shaft, individualCylinder_Weibe_Hohenberg_Biogas.crankshaft) 
          annotation (Line(
            points={{-20,-60},{3,-60},{3,-11.6133}},
            color={0,0,0},
            smooth=Smooth.None));
        connect(intake.tap, individualCylinder_Weibe_Hohenberg_Biogas.intake) 
          annotation (Line(
            points={{-50,60},{-50,42.1333},{-16,42.1333}},
            color={255,0,0},
            smooth=Smooth.None));
        connect(exhaust.tap, individualCylinder_Weibe_Hohenberg_Biogas.exhaust) 
          annotation (Line(
            points={{50,60},{50,42.1333},{22,42.1333}},
            color={255,0,0},
            smooth=Smooth.None));
        connect(individualCylinder_Weibe_Hohenberg_Biogas.bioP,
          biogasPropertySource.bioP) annotation (Line(
            points={{23.9,28.2133},{54,28.2133},{54,37},{63,37}},
            color={0,0,255},
            smooth=Smooth.None));
        annotation (
          Window(
            x=0.1,
            y=0.21,
            width=0.6,
            height=0.6),
          Documentation(info="This simulation involves a single cylinder connected
to a dynamometer.  One interesting thing to look at
in this simulation is the instantaneous torque felt
by the dynamometer vs. the average torque.  Both are
available as variables to be viewed within the dynamometer
model.
"),       Diagram(coordinateSystem(preserveAspectRatio=false,extent={{-100,
                  -100},{100,100}},
              grid={2,2}),     graphics));
      end CylinderOnDyno_Weibe_Hohenburg_Biogas;

      model I4EngineOnDyno_Biogas "I4 engine connected to a dynamometer"
        extends Modelica.Icons.Example;
        Engine.Components.Dynamometer dyno annotation (Placement(
              transformation(extent={{-100,-76},{-60,-36}}, rotation=0)));
        Engine.Components.Reservoir intake(P=101800) annotation (
                         Placement(transformation(extent={{-60,60},{-20,100}},
                rotation=0)));
        Engine.Components.Reservoir exhaust annotation (Placement(
              transformation(extent={{20,20},{60,60}}, rotation=0)));
        Engine.SportsCarGeometry sample_geometry(
          bore=.08,
          stroke=.08,
          conrod=.152,
          comp_ratio=9.5) annotation (                            Placement(
              transformation(extent={{60,-60},{100,-20}}, rotation=0)));
        Engine.Components.Manifold intake_manifold annotation (Placement(
              transformation(extent={{-60,8},{-20,48}}, rotation=0)));
        Modelica.Blocks.Sources.Constant throttle_angle(k=90)   annotation (
                                   Placement(transformation(extent={{-140,18},
                  {-120,38}}, rotation=0)));
        Modelica.Blocks.Sources.Constant rpm(k=1500)            annotation (
                                   Placement(transformation(extent={{-140,18},
                  {-120,38}}, rotation=0)));
        Engine.BiogasPropertySource biogasPropertySource(
          lhv=44700000,
          afr=14.6,
          SGF=1) 
          annotation (Placement(transformation(extent={{70,-76},{90,-56}})));
        Engine.Components.I4_Engine_Biogas i4_Engine_Biogas(spark_advance=20,
            burn_duration=60) 
          annotation (Placement(transformation(extent={{-22,-66},{24,-22}})));
      equation
        connect(intake_manifold.ambient, intake.tap) annotation (Line(
            points={{-40,48},{-40,60}},
            color={255,0,0},
            smooth=Smooth.None));
        connect(throttle_angle.y, intake_manifold.throttle_angle) annotation (
            Line(
            points={{-119,28},{-62,28}},
            color={0,0,127},
            smooth=Smooth.None));
        connect(rpm.y, dyno.rpm) annotation (Line(
            points={{-119,28},{-110.5,28},{-110.5,-56},{-102,-56}},
            color={0,0,127},
            smooth=Smooth.None));
        connect(intake_manifold.manifold, i4_Engine_Biogas.intake) annotation (
            Line(
            points={{-40,8},{-40,-28},{-22,-28},{-22,-26.4}},
            color={255,0,0},
            smooth=Smooth.None));
        connect(exhaust.tap, i4_Engine_Biogas.exhaust) annotation (Line(
            points={{40,20},{40,-28},{24,-28},{24,-26.4}},
            color={255,0,0},
            smooth=Smooth.None));
        connect(biogasPropertySource.bioP, i4_Engine_Biogas.bioP) annotation (
            Line(
            points={{69,-67},{36,-67},{36,-54.12},{25.84,-54.12}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(i4_Engine_Biogas.engine_geometry, sample_geometry.geom) 
          annotation (Line(
            points={{26.3,-44},{60,-44},{60,-40},{58,-40}},
            color={0,0,0},
            smooth=Smooth.None));
        connect(dyno.shaft, i4_Engine_Biogas.crankshaft) annotation (Line(
            points={{-60,-56},{-50,-56},{-50,-54},{-22,-54},{-22,-52.8}},
            color={0,0,0},
            smooth=Smooth.None));
        annotation (
          Window(
            x=0.32,
            y=0.07,
            width=0.6,
            height=0.6),
          Documentation(info="This is a simulation of an I4 engine connected to a dynamometer.  The interesting
thing about this simulation that sets it apart from the single cylinder case is
that this model will demonstrate manifold filling and emptying effects.
"),       Diagram(coordinateSystem(preserveAspectRatio=false,extent={{-150,
                  -100},{100,100}},
              grid={2,2}),     graphics));
      end I4EngineOnDyno_Biogas;

      model I4EngineOnDyno_Biogas_Generator
        "I4 engine connected to a Generator"
        extends Modelica.Icons.Example;
        Engine.Components.Reservoir intake(P=101800) annotation (
                         Placement(transformation(extent={{-60,60},{-20,100}},
                rotation=0)));
        Engine.Components.Reservoir exhaust annotation (Placement(
              transformation(extent={{20,20},{60,60}}, rotation=0)));
        Engine.SportsCarGeometry sample_geometry(
          bore=.08,
          stroke=.08,
          conrod=.152,
          comp_ratio=9.5) annotation (                            Placement(
              transformation(extent={{60,-60},{100,-20}}, rotation=0)));
        Engine.Components.Manifold intake_manifold annotation (Placement(
              transformation(extent={{-60,8},{-20,48}}, rotation=0)));
        Modelica.Blocks.Sources.Constant throttle_angle(k=90)   annotation (
                                   Placement(transformation(extent={{-140,18},
                  {-120,38}}, rotation=0)));
        Engine.BiogasPropertySource biogasPropertySource(
          lhv=44700000,
          afr=14.6,
          SGF=1) 
          annotation (Placement(transformation(extent={{70,-90},{90,-70}})));
        Engine.Components.I4_Engine_Biogas i4_Engine_Biogas(spark_advance=20,
            burn_duration=60) 
          annotation (Placement(transformation(extent={{-22,-66},{24,-22}})));
        EditionGasEngine.BioDES2.Generator_Port generator_Port 
          annotation (Placement(transformation(extent={{-112,-48},{-88,-24}})));
      equation
        connect(intake_manifold.ambient, intake.tap) annotation (Line(
            points={{-40,48},{-40,60}},
            color={255,0,0},
            smooth=Smooth.None));
        connect(throttle_angle.y, intake_manifold.throttle_angle) annotation (
            Line(
            points={{-119,28},{-62,28}},
            color={0,0,127},
            smooth=Smooth.None));
        connect(intake_manifold.manifold, i4_Engine_Biogas.intake) annotation (
            Line(
            points={{-40,8},{-40,-28},{-22,-28},{-22,-26.4}},
            color={255,0,0},
            smooth=Smooth.None));
        connect(exhaust.tap, i4_Engine_Biogas.exhaust) annotation (Line(
            points={{40,20},{40,-28},{24,-28},{24,-26.4}},
            color={255,0,0},
            smooth=Smooth.None));
        connect(biogasPropertySource.bioP, i4_Engine_Biogas.bioP) annotation (
            Line(
            points={{69,-81},{36,-81},{36,-54.12},{25.84,-54.12}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(i4_Engine_Biogas.engine_geometry, sample_geometry.geom) 
          annotation (Line(
            points={{26.3,-44},{60,-44},{60,-40},{58,-40}},
            color={0,0,0},
            smooth=Smooth.None));
        connect(generator_Port.flange_b1, i4_Engine_Biogas.crankshaft) 
          annotation (Line(
            points={{-88,-36.96},{-62,-36.96},{-62,-52},{-22,-52},{-22,-52.8}},
            color={0,0,0},
            smooth=Smooth.None));

        annotation (
          Window(
            x=0.32,
            y=0.07,
            width=0.6,
            height=0.6),
          Documentation(info="This is a simulation of an I4 engine connected to a dynamometer.  The interesting
thing about this simulation that sets it apart from the single cylinder case is
that this model will demonstrate manifold filling and emptying effects.
"),       Diagram(coordinateSystem(preserveAspectRatio=true, extent={{-150,
                  -100},{100,100}},
              grid={2,2}),     graphics));
      end I4EngineOnDyno_Biogas_Generator;

      model CylinderOnDyno_Weibe_Eichelberg_Biogas
        "Single cylinder engine connected to a dynamometer"
        extends Modelica.Icons.Example;
        Engine.Components.Reservoir intake(P=50000) annotation (Placement(
              transformation(extent={{-60,60},{-40,80}}, rotation=0)));
        Engine.Components.Reservoir exhaust annotation (
            Placement(transformation(extent={{40,60},{60,80}}, rotation=0)));
        Engine.Components.Dynamometer dyno annotation (
            Placement(transformation(extent={{-40,-70},{-20,-50}}, rotation=0)));
        Modelica.Blocks.Sources.Ramp speed_profile(
          height=2000,
          duration=1,
          startTime=1,
          offset=1500)   annotation (                             Placement(
              transformation(extent={{-80,-70},{-60,-50}}, rotation=0)));
        Engine.GeometrySource sample_geometry(
          bore=.08,
          stroke=.08,
          conrod=.152,
          comp_ratio=9.5) annotation (                        Placement(
              transformation(extent={{60,0},{80,20}}, rotation=0)));
        Engine.BiogasPropertySource biogasPropertySource(
          lhv=44600000,
          afr=14.6,
          SGF=1) 
          annotation (Placement(transformation(extent={{64,28},{84,48}})));
        Engine.Components.IndividualCylinder_Weibe_Eichelberg_Biogas
          individualCylinder_Weibe_Eichelberg_Biogas(spark_advance=20,
            burn_duration=60) 
          annotation (Placement(transformation(extent={{-24,-16},{16,46}})));
      equation
        connect(speed_profile.y, dyno.rpm) annotation (Line(
            points={{-59,-60},{-54.5,-60},{-54.5,-80},{-50,-80},{-50,-40},{
                -41,-40}},
            color={0,0,127},
            smooth=Smooth.None));
        connect(individualCylinder_Weibe_Eichelberg_Biogas.bioP,
          biogasPropertySource.bioP) annotation (Line(
            points={{18,26.9867},{46,26.9867},{46,37},{63,37}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(individualCylinder_Weibe_Eichelberg_Biogas.geom,
          sample_geometry.geom) annotation (Line(
            points={{18,15},{40,15},{40,10},{59,10}},
            color={0,0,0},
            smooth=Smooth.None));
        connect(dyno.shaft, individualCylinder_Weibe_Eichelberg_Biogas.crankshaft) 
          annotation (Line(
            points={{-20,-60},{-4,-60},{-4,-15.5867}},
            color={0,0,0},
            smooth=Smooth.None));
        connect(individualCylinder_Weibe_Eichelberg_Biogas.intake, intake.tap) 
          annotation (Line(
            points={{-24,41.8667},{-40,41.8667},{-40,42},{-50,42},{-50,60}},
            color={255,0,0},
            smooth=Smooth.None));
        connect(individualCylinder_Weibe_Eichelberg_Biogas.exhaust, exhaust.tap) 
          annotation (Line(
            points={{16,41.8667},{38,41.8667},{38,42},{50,42},{50,60}},
            color={255,0,0},
            smooth=Smooth.None));
        annotation (
          Window(
            x=0.1,
            y=0.21,
            width=0.6,
            height=0.6),
          Documentation(info="This simulation involves a single cylinder connected
to a dynamometer.  One interesting thing to look at
in this simulation is the instantaneous torque felt
by the dynamometer vs. the average torque.  Both are
available as variables to be viewed within the dynamometer
model.
"),       Diagram(coordinateSystem(preserveAspectRatio=false,extent={{-100,
                  -100},{100,100}},
              grid={2,2}),     graphics));
      end CylinderOnDyno_Weibe_Eichelberg_Biogas;

      model I4EngineOnDyno_Biogas_Eichelberg
        "I4 engine connected to a dynamometer"
        extends Modelica.Icons.Example;
        Engine.Components.Dynamometer dyno annotation (Placement(
              transformation(extent={{-100,-76},{-60,-36}}, rotation=0)));
        Engine.Components.Reservoir intake(P=101800) annotation (
                         Placement(transformation(extent={{-60,60},{-20,100}},
                rotation=0)));
        Engine.Components.Reservoir exhaust annotation (Placement(
              transformation(extent={{20,20},{60,60}}, rotation=0)));
        Engine.SportsCarGeometry sample_geometry(
          bore=.08,
          stroke=.08,
          conrod=.152,
          comp_ratio=9.5) annotation (                            Placement(
              transformation(extent={{60,-60},{100,-20}}, rotation=0)));
        Engine.Components.Manifold intake_manifold annotation (Placement(
              transformation(extent={{-60,8},{-20,48}}, rotation=0)));
        Modelica.Blocks.Sources.Constant throttle_angle(k=90)   annotation (
                                   Placement(transformation(extent={{-140,18},
                  {-120,38}}, rotation=0)));
        Modelica.Blocks.Sources.Constant rpm(k=1500)            annotation (
                                   Placement(transformation(extent={{-140,18},
                  {-120,38}}, rotation=0)));
        Engine.BiogasPropertySource biogasPropertySource(
          lhv=44700000,
          afr=14.6,
          SGF=1) 
          annotation (Placement(transformation(extent={{70,-76},{90,-56}})));
        Engine.Components.I4_Engine_Biogas_Eichelberg
          i4_Engine_Biogas_Eichelberg(
            spark_advance=20, burn_duration=60) 
          annotation (Placement(transformation(extent={{-20,-58},{26,-12}})));
      equation
        connect(intake_manifold.ambient, intake.tap) annotation (Line(
            points={{-40,48},{-40,60}},
            color={255,0,0},
            smooth=Smooth.None));
        connect(throttle_angle.y, intake_manifold.throttle_angle) annotation (
            Line(
            points={{-119,28},{-62,28}},
            color={0,0,127},
            smooth=Smooth.None));
        connect(rpm.y, dyno.rpm) annotation (Line(
            points={{-119,28},{-110.5,28},{-110.5,-56},{-102,-56}},
            color={0,0,127},
            smooth=Smooth.None));
        connect(intake_manifold.manifold, i4_Engine_Biogas_Eichelberg.intake) 
          annotation (Line(
            points={{-40,8},{-40,-16.6},{-20,-16.6}},
            color={255,0,0},
            smooth=Smooth.None));
        connect(i4_Engine_Biogas_Eichelberg.exhaust, exhaust.tap) annotation (
            Line(
            points={{26,-16.6},{34,-16.6},{34,-16},{40,-16},{40,20}},
            color={255,0,0},
            smooth=Smooth.None));
        connect(dyno.shaft, i4_Engine_Biogas_Eichelberg.crankshaft) annotation (
           Line(
            points={{-60,-56},{-30,-56},{-30,-44.2},{-20,-44.2}},
            color={0,0,0},
            smooth=Smooth.None));
        connect(i4_Engine_Biogas_Eichelberg.bioP, biogasPropertySource.bioP) 
          annotation (Line(
            points={{27.84,-45.58},{44,-45.58},{44,-67},{69,-67}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(i4_Engine_Biogas_Eichelberg.engine_geometry, sample_geometry.geom) 
          annotation (Line(
            points={{28.3,-35},{56,-35},{56,-40},{58,-40}},
            color={0,0,0},
            smooth=Smooth.None));
        annotation (
          Window(
            x=0.32,
            y=0.07,
            width=0.6,
            height=0.6),
          Documentation(info="This is a simulation of an I4 engine connected to a dynamometer.  The interesting
thing about this simulation that sets it apart from the single cylinder case is
that this model will demonstrate manifold filling and emptying effects.
"),       Diagram(coordinateSystem(preserveAspectRatio=false,extent={{-150,
                  -100},{100,100}},
              grid={2,2}),     graphics));
      end I4EngineOnDyno_Biogas_Eichelberg;

      model CylinderOnDyno_Weibe_Eichelberg_NaturalGas
        "Single cylinder engine connected to a dynamometer"
        extends Modelica.Icons.Example;
        Engine.Components.Reservoir intake(P=100000) 
                                                    annotation (Placement(
              transformation(extent={{-60,60},{-40,80}}, rotation=0)));
        Engine.Components.Reservoir exhaust annotation (
            Placement(transformation(extent={{40,60},{60,80}}, rotation=0)));
        Engine.Components.Dynamometer dyno annotation (
            Placement(transformation(extent={{-40,-70},{-20,-50}}, rotation=0)));
        Modelica.Blocks.Sources.Ramp speed_profile(
          height=2000,
          duration=1,
          startTime=1,
          offset=1500)   annotation (                             Placement(
              transformation(extent={{-80,-70},{-60,-50}}, rotation=0)));
        Engine.GeometrySource sample_geometry(
          bore=.062,
          stroke=.066,
          comp_ratio=8.7,
          conrod=.035)    annotation (                        Placement(
              transformation(extent={{60,0},{80,20}}, rotation=0)));
        Engine.BiogasPropertySource biogasPropertySource(
          lhv=34020000,
          afr=9.52,
          SGF=0.552) 
          annotation (Placement(transformation(extent={{64,28},{84,48}})));
        Engine.Components.IndividualCylinder_Weibe_Eichelberg_Biogas
          individualCylinder_Weibe_Eichelberg_Biogas(
          spark_advance=5,
          burn_duration=40,
          ivo=170.5,
          evc=189.5,
          ivc=295.5,
          evo=64.5) 
          annotation (Placement(transformation(extent={{-24,-16},{16,46}})));
      equation
        connect(speed_profile.y, dyno.rpm) annotation (Line(
            points={{-59,-60},{-54.5,-60},{-54.5,-80},{-50,-80},{-50,-40},{
                -41,-40}},
            color={0,0,127},
            smooth=Smooth.None));
        connect(individualCylinder_Weibe_Eichelberg_Biogas.bioP,
          biogasPropertySource.bioP) annotation (Line(
            points={{18,26.9867},{46,26.9867},{46,37},{63,37}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(individualCylinder_Weibe_Eichelberg_Biogas.geom,
          sample_geometry.geom) annotation (Line(
            points={{18,15},{40,15},{40,10},{59,10}},
            color={0,0,0},
            smooth=Smooth.None));
        connect(dyno.shaft, individualCylinder_Weibe_Eichelberg_Biogas.crankshaft) 
          annotation (Line(
            points={{-20,-60},{-4,-60},{-4,-15.5867}},
            color={0,0,0},
            smooth=Smooth.None));
        connect(individualCylinder_Weibe_Eichelberg_Biogas.intake, intake.tap) 
          annotation (Line(
            points={{-24,41.8667},{-40,41.8667},{-40,42},{-50,42},{-50,60}},
            color={255,0,0},
            smooth=Smooth.None));
        connect(individualCylinder_Weibe_Eichelberg_Biogas.exhaust, exhaust.tap) 
          annotation (Line(
            points={{16,41.8667},{38,41.8667},{38,42},{50,42},{50,60}},
            color={255,0,0},
            smooth=Smooth.None));
        annotation (
          Window(
            x=0.1,
            y=0.21,
            width=0.6,
            height=0.6),
          Documentation(info="This simulation involves a single cylinder connected
to a dynamometer.  One interesting thing to look at
in this simulation is the instantaneous torque felt
by the dynamometer vs. the average torque.  Both are
available as variables to be viewed within the dynamometer
model.
"),       Diagram(coordinateSystem(preserveAspectRatio=false,extent={{-100,
                  -100},{100,100}},
              grid={2,2}),     graphics));
      end CylinderOnDyno_Weibe_Eichelberg_NaturalGas;

      model CylinderOnDyno_Weibe_Eichelberg_NaturalGas_faiair
        "Single cylinder engine connected to a dynamometer"
        extends Modelica.Icons.Example;
        Engine.Components.Reservoir intake(P=50000) annotation (Placement(
              transformation(extent={{-60,60},{-40,80}}, rotation=0)));
        Engine.Components.Reservoir exhaust annotation (
            Placement(transformation(extent={{40,60},{60,80}}, rotation=0)));
        Engine.Components.Dynamometer dyno annotation (
            Placement(transformation(extent={{-40,-70},{-20,-50}}, rotation=0)));
        Modelica.Blocks.Sources.Ramp speed_profile(
          duration=1,
          startTime=1,
          height=1000,
          offset=1500)   annotation (                             Placement(
              transformation(extent={{-80,-70},{-60,-50}}, rotation=0)));
        Engine.GeometrySource sample_geometry(
          bore=.062,
          stroke=.066,
          comp_ratio=8.7,
          conrod=.112)    annotation (                        Placement(
              transformation(extent={{60,0},{80,20}}, rotation=0)));
        Engine.BiogasPropertySource biogasPropertySource(
          SGF=0.552,
          lhv=49000000,
          afr=16.765) 
          annotation (Placement(transformation(extent={{66,28},{86,48}})));
        Engine.Components.IndividualCylinder_Weibe_Eichelberg_Biogas
          individualCylinder_Weibe_Eichelberg_Biogas(
          spark_advance=5,
          burn_duration=40,
          ivo=170.5,
          evc=189.5,
          ivc=295.5,
          evo=64.5) 
          annotation (Placement(transformation(extent={{-24,-16},{16,46}})));
      equation
        connect(speed_profile.y, dyno.rpm) annotation (Line(
            points={{-59,-60},{-54.5,-60},{-54.5,-80},{-50,-80},{-50,-40},{
                -41,-40}},
            color={0,0,127},
            smooth=Smooth.None));
        connect(individualCylinder_Weibe_Eichelberg_Biogas.bioP,
          biogasPropertySource.bioP) annotation (Line(
            points={{18,26.9867},{46,26.9867},{46,37},{65,37}},
            color={0,0,255},
            smooth=Smooth.None));
        connect(individualCylinder_Weibe_Eichelberg_Biogas.geom,
          sample_geometry.geom) annotation (Line(
            points={{18,15},{40,15},{40,10},{59,10}},
            color={0,0,0},
            smooth=Smooth.None));
        connect(dyno.shaft, individualCylinder_Weibe_Eichelberg_Biogas.crankshaft) 
          annotation (Line(
            points={{-20,-60},{-4,-60},{-4,-15.5867}},
            color={0,0,0},
            smooth=Smooth.None));
        connect(individualCylinder_Weibe_Eichelberg_Biogas.intake, intake.tap) 
          annotation (Line(
            points={{-24,41.8667},{-40,41.8667},{-40,42},{-50,42},{-50,60}},
            color={255,0,0},
            smooth=Smooth.None));
        connect(individualCylinder_Weibe_Eichelberg_Biogas.exhaust, exhaust.tap) 
          annotation (Line(
            points={{16,41.8667},{38,41.8667},{38,42},{50,42},{50,60}},
            color={255,0,0},
            smooth=Smooth.None));
        annotation (
          Window(
            x=0.1,
            y=0.21,
            width=0.6,
            height=0.6),
          Documentation(info="This simulation involves a single cylinder connected
to a dynamometer.  One interesting thing to look at
in this simulation is the instantaneous torque felt
by the dynamometer vs. the average torque.  Both are
available as variables to be viewed within the dynamometer
model.
"),       Diagram(coordinateSystem(preserveAspectRatio=false,extent={{-100,
                  -100},{100,100}},
              grid={2,2}),     graphics));
      end CylinderOnDyno_Weibe_Eichelberg_NaturalGas_faiair;
      annotation (
        Window(
          x=0.04,
          y=0.32,
          width=0.28,
          height=0.42,
          library=1,
          autolayout=1),
        Documentation(info="This package contains several example models that
demonstrate the component and subsystem models
found in the 'SimpleCar' package.  Each of the examples
includes some documentation on how the examples
should be used.
"));
    end TestSimulation;

    package Interfaces "Collection of interfaces for the 'SimpleCar' package"
      extends Modelica.Icons.Library;
      connector EngineGeometryRequired
        "Connector for components or subsystems that require geometry information"
        input Modelica.SIunits.Length bore "Engine bore";
        input Modelica.SIunits.Length stroke "Engine stroke";
        input Modelica.SIunits.Length conrod "Connecting rod length";
        input Modelica.SIunits.Volume Vc "Clearance volume";
        input Modelica.SIunits.Area Ap "Piston area";
        input Modelica.SIunits.Volume Vd "Displaced volume";
        input Modelica.SIunits.Length crank "Crank length";
        annotation (
          Diagram(coordinateSystem(
              preserveAspectRatio=false,
              extent={{-100,-100},{100,100}},
              grid={2,2}), graphics),
          Window(
            x=0.39,
            y=0.34,
            width=0.6,
            height=0.6),
          Icon(coordinateSystem(
              preserveAspectRatio=false,
              extent={{-100,-100},{100,100}},
              grid={2,2}), graphics={
              Polygon(
                points={{-100,100},{0,0},{-100,-100},{-100,98},{-100,100}},
                lineColor={0,0,0},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid),
              Polygon(
                points={{-100,100},{100,100},{100,-100},{-100,-100},{0,0},{
                    -100,100}},
                lineColor={0,0,0},
                fillColor={192,192,192},
                fillPattern=FillPattern.Solid),
              Text(extent={{-72,-72},{100,-100}}, textString=
                                                      "%name")}),
          Documentation(info="This connector should be declared for each component that requires
information about engine geometry.
"));
      end EngineGeometryRequired;

      connector EngineGeometryProvided
        "Connector to provide geometry information"

        output Modelica.SIunits.Length bore "Engine bore";
        output Modelica.SIunits.Length stroke "Engine stroke";
        output Modelica.SIunits.Length conrod "Connecting rod length";
        output Modelica.SIunits.Volume Vc "Clearance volume";
        output Modelica.SIunits.Area Ap "Piston area";
        output Modelica.SIunits.Volume Vd "Displaced volume";
        output Modelica.SIunits.Length crank "Crank length";
        annotation (
          Icon(coordinateSystem(
              preserveAspectRatio=false,
              extent={{-100,-100},{100,100}},
              grid={2,2}), graphics={Polygon(
                points={{-100,100},{0,0},{-100,-100},{-100,98},{-100,100}},
                lineColor={0,0,0},
                fillColor={192,192,192},
                fillPattern=FillPattern.Solid), Polygon(
                points={{-100,100},{100,100},{100,-100},{-100,-100},{0,0},{
                    -100,100}},
                lineColor={0,0,0},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid)}),
          Window(
            x=0.3,
            y=0.14,
            width=0.6,
            height=0.6),
          Documentation(info="This component is used with components that
provide geometry information.
"));
      end EngineGeometryProvided;

      connector Gas "Thermodynamic connector"
        Modelica.SIunits.Pressure P "Gas pressure";
        Modelica.SIunits.Temperature T "Gas temperature";
        flow Modelica.SIunits.MassFlowRate mdot "Mass flow rate";
        flow Modelica.SIunits.HeatFlowRate q "Heat flow rate";
        annotation (
          Icon(coordinateSystem(
              preserveAspectRatio=false,
              extent={{-100,-100},{100,100}},
              grid={2,2}), graphics={Rectangle(
                extent={{-100,100},{100,-100}},
                lineColor={255,0,0},
                fillColor={255,191,127},
                fillPattern=FillPattern.Solid), Text(
                extent={{-100,-100},{100,-136}},
                lineColor={255,127,0},
                textString=
                     "%name")}),
          Window(
            x=0.22,
            y=0.1,
            width=0.3,
            height=0.75),
          Diagram(coordinateSystem(
              preserveAspectRatio=false,
              extent={{-100,-100},{100,100}},
              grid={2,2}), graphics),
          Documentation(info="This connector contains the potential and flow
variables associated with the gas moving
through the engine.  These potentials and flows
are from both the energy and mass domains.
"));
      end Gas;

      connector GearSelectorOutput "Controller indicated gear selection"
        output Integer gear;
        annotation (
          Icon(coordinateSystem(
              preserveAspectRatio=false,
              extent={{-100,-100},{100,100}},
              grid={2,2}), graphics={Polygon(
                points={{-100,100},{100,0},{-100,-100},{-100,100}},
                lineColor={127,0,255},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid), Line(points={{60,20},{60,-20}},
                  color={127,0,255})}),
          Window(
            x=0.27,
            y=0.32,
            width=0.6,
            height=0.6),
          Documentation(info="This connector is an output from any transmission shifting
model.
"));
      end GearSelectorOutput;

      connector GearSelectorInput "Gear selection input for transmissions"
        input Integer gear;
        annotation (
          Icon(coordinateSystem(
              preserveAspectRatio=false,
              extent={{-100,-100},{100,100}},
              grid={2,2}), graphics={Polygon(
                points={{-100,100},{100,0},{-100,-100},{-100,100}},
                lineColor={127,0,255},
                fillColor={191,127,255},
                fillPattern=FillPattern.Solid), Polygon(
                points={{60,20},{60,-20},{64,-18},{100,0},{60,20}},
                lineColor={127,0,255},
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid)}),
          Window(
            x=0.39,
            y=0.34,
            width=0.6,
            height=0.6),
          Documentation(info="This connectoris used as an input for transmissions.  The connector
contains information about what gear the transmission should be in.
"));
      end GearSelectorInput;

      partial model Transmission "Transmission Interface"
        Interfaces.GearSelectorInput gear_selector annotation (Placement(
              transformation(
              origin={0,110},
              extent={{-10,-10},{10,10}},
              rotation=270)));
        Modelica.Mechanics.Rotational.Interfaces.Flange_a driveline annotation (Placement(
              transformation(extent={{-110,-10},{-90,10}}, rotation=0)));
        Modelica.Mechanics.Rotational.Interfaces.Flange_b engine annotation (Placement(
              transformation(extent={{90,-10},{110,10}}, rotation=0)));
        annotation (
          Window(
            x=0.39,
            y=0.34,
            width=0.6,
            height=0.6),
          Documentation(info="The transmission interface contains a connection to the engine, a connection to the
driveline and a gear selector input.  This interface is defined so that it can
be used in conjunction with replaceable declarations where the transmission interface
is the constraining type.
"));
      end Transmission;

      partial model ShiftStrategy "Shift strategy interface"
        parameter Modelica.SIunits.Length tire_radius "Tire radius";
      protected
        Types.KilometersPerHour kph "Vehicle speed";
      public
        Interfaces.GearSelectorOutput gear_request annotation (Placement(
              transformation(
              origin={0,-110},
              extent={{-10,-10},{10,10}},
              rotation=270)));
        Modelica.Mechanics.Rotational.Interfaces.Flange_a wheel annotation (Placement(
              transformation(extent={{-110,-10},{-90,10}}, rotation=0)));
      equation
        kph = der(wheel.phi)*tire_radius*60*60/1000;
        wheel.tau = 0;
        annotation (
          Icon(coordinateSystem(
              preserveAspectRatio=false,
              extent={{-100,-100},{100,100}},
              grid={2,2}), graphics={
              Ellipse(
                extent={{-80,80},{0,0}},
                lineColor={0,0,0},
                fillColor={192,192,192},
                fillPattern=FillPattern.Solid),
              Line(points={{-74,20},{-40,40}}, color={0,0,0}),
              Line(points={{-80,40},{-40,40}}, color={0,0,0}),
              Line(points={{-74,60},{-40,40}}, color={0,0,0}),
              Line(points={{-60,74},{-40,40}}, color={0,0,0}),
              Line(points={{-40,80},{-40,42}}, color={0,0,0}),
              Line(points={{-20,74},{-40,40}}, color={0,0,0}),
              Line(points={{-6,60},{-40,40}}, color={255,0,0}),
              Line(points={{0,40},{-40,40}}, color={255,0,0}),
              Ellipse(
                extent={{-70,70},{-10,10}},
                lineColor={0,0,255},
                pattern=LinePattern.None,
                fillColor={192,192,192},
                fillPattern=FillPattern.Solid),
              Line(points={{-40,40},{-60,60}}, color={0,0,0}),
              Polygon(
                points={{-60,60},{-58,52},{-52,58},{-60,60}},
                lineColor={0,0,0},
                fillColor={0,0,0},
                fillPattern=FillPattern.Solid),
              Ellipse(
                extent={{-38,42},{-42,38}},
                lineColor={0,0,0},
                fillColor={0,0,0},
                fillPattern=FillPattern.Solid),
              Line(
                points={{-90,0},{-40,0}},
                color={0,0,0},
                thickness=0.5),
              Line(
                points={{0,40},{0,-100}},
                color={127,0,255},
                thickness=0.5),
              Text(extent={{-80,-20},{80,-80}}, textString=
                                                    "%name")}),
          Window(
            x=0.39,
            y=0.34,
            width=0.6,
            height=0.6),
          Documentation(info="This is the basic interface for any shift strategy model.  It connects to the
axle of the car and, using the tire radius, computes the vehicles translational
speed. This speed is then used to determine the appropriate gear which is then
assigned to the output gear selector.
"));
      end ShiftStrategy;

      partial model Chassis "Generic chassis interface"
        Modelica.Mechanics.Translational.Interfaces.Flange_a road
          "Road contact" 
          annotation (Placement(transformation(extent={{-50,-110},{-30,-90}},
                rotation=0)));
        Modelica.Mechanics.Rotational.Interfaces.Flange_a power "Driveline" 
          annotation (Placement(transformation(extent={{70,-10},{90,10}},
                rotation=0)));
        Modelica.Blocks.Interfaces.OutPort speed(final n=1) annotation (Placement(
              transformation(
              origin={60,-110},
              extent={{-10,-10},{10,10}},
              rotation=270)));
        Modelica.Mechanics.Rotational.Interfaces.Flange_a wheel annotation (Placement(
              transformation(extent={{-30,-70},{-10,-50}}, rotation=0)));
        annotation (
          Window(
            x=0.39,
            y=0.34,
            width=0.6,
            height=0.6),
          Documentation(info="This chassis interface has connections for the driveline side of the transmission, the wheels, road and a vehicle speed output.
This interface can be used as the constraining type in a replaceable declaration to allow easy substitution of chassis models."));
      end Chassis;

      partial model Cylinder
        Modelica.Mechanics.Rotational.Interfaces.Flange_a crankshaft annotation (Placement(
              transformation(extent={{-10,-208},{10,-188}}, rotation=0)));
        Interfaces.Gas intake annotation (Placement(transformation(extent={{
                  -110,70},{-90,90}}, rotation=0)));
        Interfaces.Gas exhaust annotation (Placement(transformation(extent={{
                  90,70},{110,90}}, rotation=0)));
        Interfaces.EngineGeometryRequired geom annotation (Placement(
              transformation(extent={{120,-60},{100,-40}}, rotation=0)));
      end Cylinder;

      partial model Engine "Generic engine interface"
        Modelica.Mechanics.Rotational.Interfaces.Flange_a crankshaft annotation (Placement(
              transformation(extent={{-110,-50},{-90,-30}}, rotation=0)));
        Interfaces.Gas intake annotation (Placement(transformation(extent={{
                  -110,70},{-90,90}}, rotation=0)));
        Interfaces.Gas exhaust annotation (Placement(transformation(extent={{
                  90,70},{110,90}}, rotation=0)));
        Interfaces.EngineGeometryRequired engine_geometry annotation (Placement(
              transformation(
              origin={110,0},
              extent={{-10,-10},{10,10}},
              rotation=180)));
        annotation (
          Window(
            x=0.21,
            y=0.19,
            width=0.6,
            height=0.6),
          Documentation(info="This generic engine interface includes a connection for the crankshaft, the intake system, the exhause system and an input
for engine geometry information.  This interface can be used as the constraining type on replaceable declaration to allow
easy substitution of engine models.
"));
      end Engine;

      connector BiogasPropertyRequired
        "Connector for components or subsystems that require property information"

        input Modelica.SIunits.SpecificEnergy lhv "Lower heating value";
        input Real afr "Stoichiometric Air/Fuel Ratio";
        input Real SGF "Specific Gravity of Fuel";

         annotation (
          Diagram(coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},
                  {100,100}}), graphics),
          Coordsys(
            extent=[-100, -100; 100, 100],
            grid=[2, 2],
            component=[20, 20]),
          Window(
            x=0.39,
            y=0.34,
            width=0.6,
            height=0.6),
          Icon(
            coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},{100,
                  100}}),
            Text(extent=[-72, -72; 100, -100], string="%name"),
            graphics={
              Line(
                points={{12,94},{-44,78},{-36,38},{-66,30},{-82,-16},{-58,-48},
                    {-36,-64},{-22,-80},{4,-56},{52,-78},{78,-70},{98,-58},{76,
                    -10},{100,2},{100,30},{74,38},{64,100},{20,84},{12,94}},
                color={0,0,255},
                smooth=Smooth.Bezier),
              Ellipse(
                extent={{28,-36},{34,-42}},
                lineColor={0,0,255},
                fillPattern=FillPattern.Solid,
                fillColor={85,255,255}),
              Ellipse(
                extent={{42,-36},{48,-42}},
                lineColor={0,0,255},
                fillPattern=FillPattern.Solid,
                fillColor={85,255,255}),
              Ellipse(
                extent={{44,-20},{50,-26}},
                lineColor={0,0,255},
                fillPattern=FillPattern.Solid,
                fillColor={85,255,255}),
              Ellipse(
                extent={{30,-22},{46,-38}},
                lineColor={0,0,255},
                fillPattern=FillPattern.Solid,
                fillColor={0,0,0}),
              Ellipse(
                extent={{-16,-30},{-10,-36}},
                lineColor={0,0,255},
                fillPattern=FillPattern.Solid,
                fillColor={255,0,0}),
              Ellipse(
                extent={{26,-20},{32,-26}},
                lineColor={0,0,255},
                fillPattern=FillPattern.Solid,
                fillColor={85,255,255}),
              Ellipse(
                extent={{-32,-24},{-16,-40}},
                lineColor={0,0,255},
                fillPattern=FillPattern.Solid,
                fillColor={0,0,0}),
              Ellipse(
                extent={{72,-54},{78,-60}},
                lineColor={0,0,255},
                fillPattern=FillPattern.Solid,
                fillColor={255,0,0}),
              Ellipse(
                extent={{56,-48},{72,-64}},
                lineColor={0,0,255},
                fillPattern=FillPattern.Solid,
                fillColor={0,0,0}),
              Ellipse(
                extent={{64,-42},{70,-48}},
                lineColor={0,0,255},
                fillPattern=FillPattern.Solid,
                fillColor={255,0,0}),
              Ellipse(
                extent={{40,62},{46,56}},
                lineColor={0,0,255},
                fillPattern=FillPattern.Solid,
                fillColor={255,0,0}),
              Ellipse(
                extent={{24,68},{40,52}},
                lineColor={0,0,255},
                fillPattern=FillPattern.Solid,
                fillColor={0,0,0}),
              Ellipse(
                extent={{32,74},{38,68}},
                lineColor={0,0,255},
                fillPattern=FillPattern.Solid,
                fillColor={255,0,0}),
              Ellipse(
                extent={{-28,40},{-22,34}},
                lineColor={0,0,255},
                fillPattern=FillPattern.Solid,
                fillColor={85,255,255}),
              Ellipse(
                extent={{-14,40},{-8,34}},
                lineColor={0,0,255},
                fillPattern=FillPattern.Solid,
                fillColor={85,255,255}),
              Ellipse(
                extent={{-12,56},{-6,50}},
                lineColor={0,0,255},
                fillPattern=FillPattern.Solid,
                fillColor={85,255,255}),
              Ellipse(
                extent={{-26,54},{-10,38}},
                lineColor={0,0,255},
                fillPattern=FillPattern.Solid,
                fillColor={0,0,0}),
              Ellipse(
                extent={{-30,56},{-24,50}},
                lineColor={0,0,255},
                fillPattern=FillPattern.Solid,
                fillColor={85,255,255}),
              Ellipse(
                extent={{-52,0},{-46,-6}},
                lineColor={0,0,255},
                fillPattern=FillPattern.Solid,
                fillColor={85,255,255}),
              Ellipse(
                extent={{-38,0},{-32,-6}},
                lineColor={0,0,255},
                fillPattern=FillPattern.Solid,
                fillColor={85,255,255}),
              Ellipse(
                extent={{-36,16},{-30,10}},
                lineColor={0,0,255},
                fillPattern=FillPattern.Solid,
                fillColor={85,255,255}),
              Ellipse(
                extent={{-50,14},{-34,-2}},
                lineColor={0,0,255},
                fillPattern=FillPattern.Solid,
                fillColor={0,0,0}),
              Ellipse(
                extent={{-54,16},{-48,10}},
                lineColor={0,0,255},
                fillPattern=FillPattern.Solid,
                fillColor={85,255,255}),
              Ellipse(
                extent={{8,10},{14,4}},
                lineColor={0,0,255},
                fillPattern=FillPattern.Solid,
                fillColor={85,255,255}),
              Ellipse(
                extent={{22,10},{28,4}},
                lineColor={0,0,255},
                fillPattern=FillPattern.Solid,
                fillColor={85,255,255}),
              Ellipse(
                extent={{24,26},{30,20}},
                lineColor={0,0,255},
                fillPattern=FillPattern.Solid,
                fillColor={85,255,255}),
              Ellipse(
                extent={{10,24},{26,8}},
                lineColor={0,0,255},
                fillPattern=FillPattern.Solid,
                fillColor={0,0,0}),
              Ellipse(
                extent={{6,26},{12,20}},
                lineColor={0,0,255},
                fillPattern=FillPattern.Solid,
                fillColor={85,255,255})}),
          Documentation(info="This connector should be declared for each component that requires
information about engine geometry.
"));
      end BiogasPropertyRequired;

      connector BiogasPropertyProvided
        "Connector to provide biogas property information"

        output Modelica.SIunits.SpecificEnergy lhv "Lower heating value";
        output Real afr "Stoichiometric Air/Fuel Ratio";
        output Real SGF "Specific Gravity of Fuel";
         annotation (
          Icon(coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},{
                  100,100}}), graphics={
              Line(
                points={{12,94},{-44,78},{-36,38},{-66,30},{-82,-16},{-58,-48},
                    {-36,-64},{-22,-80},{4,-56},{52,-78},{78,-70},{98,-58},{76,
                    -10},{100,2},{100,30},{74,38},{64,100},{20,84},{12,94}},
                color={0,0,255},
                smooth=Smooth.Bezier),
              Ellipse(
                extent={{28,-36},{34,-42}},
                lineColor={0,0,255},
                fillPattern=FillPattern.Solid,
                fillColor={85,255,255}),
              Ellipse(
                extent={{42,-36},{48,-42}},
                lineColor={0,0,255},
                fillPattern=FillPattern.Solid,
                fillColor={85,255,255}),
              Ellipse(
                extent={{44,-20},{50,-26}},
                lineColor={0,0,255},
                fillPattern=FillPattern.Solid,
                fillColor={85,255,255}),
              Ellipse(
                extent={{30,-22},{46,-38}},
                lineColor={0,0,255},
                fillPattern=FillPattern.Solid,
                fillColor={0,0,0}),
              Ellipse(
                extent={{-16,-30},{-10,-36}},
                lineColor={0,0,255},
                fillPattern=FillPattern.Solid,
                fillColor={255,0,0}),
              Ellipse(
                extent={{26,-20},{32,-26}},
                lineColor={0,0,255},
                fillPattern=FillPattern.Solid,
                fillColor={85,255,255}),
              Ellipse(
                extent={{-32,-24},{-16,-40}},
                lineColor={0,0,255},
                fillPattern=FillPattern.Solid,
                fillColor={0,0,0}),
              Ellipse(
                extent={{72,-54},{78,-60}},
                lineColor={0,0,255},
                fillPattern=FillPattern.Solid,
                fillColor={255,0,0}),
              Ellipse(
                extent={{56,-48},{72,-64}},
                lineColor={0,0,255},
                fillPattern=FillPattern.Solid,
                fillColor={0,0,0}),
              Ellipse(
                extent={{64,-42},{70,-48}},
                lineColor={0,0,255},
                fillPattern=FillPattern.Solid,
                fillColor={255,0,0}),
              Ellipse(
                extent={{40,62},{46,56}},
                lineColor={0,0,255},
                fillPattern=FillPattern.Solid,
                fillColor={255,0,0}),
              Ellipse(
                extent={{24,68},{40,52}},
                lineColor={0,0,255},
                fillPattern=FillPattern.Solid,
                fillColor={0,0,0}),
              Ellipse(
                extent={{32,74},{38,68}},
                lineColor={0,0,255},
                fillPattern=FillPattern.Solid,
                fillColor={255,0,0}),
              Ellipse(
                extent={{-28,40},{-22,34}},
                lineColor={0,0,255},
                fillPattern=FillPattern.Solid,
                fillColor={85,255,255}),
              Ellipse(
                extent={{-14,40},{-8,34}},
                lineColor={0,0,255},
                fillPattern=FillPattern.Solid,
                fillColor={85,255,255}),
              Ellipse(
                extent={{-12,56},{-6,50}},
                lineColor={0,0,255},
                fillPattern=FillPattern.Solid,
                fillColor={85,255,255}),
              Ellipse(
                extent={{-26,54},{-10,38}},
                lineColor={0,0,255},
                fillPattern=FillPattern.Solid,
                fillColor={0,0,0}),
              Ellipse(
                extent={{-30,56},{-24,50}},
                lineColor={0,0,255},
                fillPattern=FillPattern.Solid,
                fillColor={85,255,255}),
              Ellipse(
                extent={{-52,0},{-46,-6}},
                lineColor={0,0,255},
                fillPattern=FillPattern.Solid,
                fillColor={85,255,255}),
              Ellipse(
                extent={{-38,0},{-32,-6}},
                lineColor={0,0,255},
                fillPattern=FillPattern.Solid,
                fillColor={85,255,255}),
              Ellipse(
                extent={{-36,16},{-30,10}},
                lineColor={0,0,255},
                fillPattern=FillPattern.Solid,
                fillColor={85,255,255}),
              Ellipse(
                extent={{-50,14},{-34,-2}},
                lineColor={0,0,255},
                fillPattern=FillPattern.Solid,
                fillColor={0,0,0}),
              Ellipse(
                extent={{-54,16},{-48,10}},
                lineColor={0,0,255},
                fillPattern=FillPattern.Solid,
                fillColor={85,255,255}),
              Ellipse(
                extent={{8,10},{14,4}},
                lineColor={0,0,255},
                fillPattern=FillPattern.Solid,
                fillColor={85,255,255}),
              Ellipse(
                extent={{22,10},{28,4}},
                lineColor={0,0,255},
                fillPattern=FillPattern.Solid,
                fillColor={85,255,255}),
              Ellipse(
                extent={{24,26},{30,20}},
                lineColor={0,0,255},
                fillPattern=FillPattern.Solid,
                fillColor={85,255,255}),
              Ellipse(
                extent={{10,24},{26,8}},
                lineColor={0,0,255},
                fillPattern=FillPattern.Solid,
                fillColor={0,0,0}),
              Ellipse(
                extent={{6,26},{12,20}},
                lineColor={0,0,255},
                fillPattern=FillPattern.Solid,
                fillColor={85,255,255})}),
          Coordsys(
            extent=[-100, -100; 100, 100],
            grid=[2, 2],
            component=[20, 20]),
          Window(
            x=0.3,
            y=0.14,
            width=0.6,
            height=0.6),
          Documentation(info="This component is used with components that
provide geometry information.
"),       Diagram(coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},
                  {100,100}}), graphics));
      end BiogasPropertyProvided;
      annotation (
        Window(
          x=0.01,
          y=0.54,
          width=0.4,
          height=0.4,
          library=1,
          autolayout=1),
        Documentation(info="This package contains numerous connector definitions and a few partial model 
definitions for the major vehicle subsystems.
"));
    end Interfaces;

    package Tests "Models to test various models"
      extends Modelica.Icons.Example;

      model SingleControlVolume "Test a single control volume"
        extends Modelica.Icons.Example;
        Engine.Components.ControlVolume control_volume annotation (
                         Placement(transformation(extent={{20,-20},{60,20}},
                rotation=0)));
        Modelica.Blocks.Sources.Sine volume(amplitude=.25, offset=1) 
          annotation (Placement(transformation(extent={{-60,-80},{-20,-40}},
                rotation=0)));
      equation
        connect(volume.y, control_volume.volume) annotation (Line(
            points={{-18,-60},{80,-60},{80,0},{62,0}},
            color={0,0,127},
            smooth=Smooth.None));
        annotation (
          Window(
            x=0.17,
            y=0.07,
            width=0.6,
            height=0.6),
          Documentation(info="This model looks at what happens to the states inside a control volume when the volume is change sinusoidally
(closely approximating the volume change in a combustion chamber).
"),       Diagram(coordinateSystem(preserveAspectRatio=false,extent={{-100,
                  -100},{100,100}},
              grid={2,2}),     graphics));
      end SingleControlVolume;

      model TestValve "Test an engine valve"
        extends Modelica.Icons.Example;
        Engine.Components.Reservoir upstream annotation (Placement(
              transformation(extent={{-75,34},{-55,54}}, rotation=0)));
        Engine.Components.Valve valve annotation (Placement(transformation(
                extent={{-23,17},{-3,37}}, rotation=0)));
        Engine.Components.ControlVolume control_volume annotation (Placement(
              transformation(extent={{-23,-25},{-3,-5}}, rotation=0)));
        Modelica.Blocks.Sources.Constant volume(k=0.004)   annotation (Placement(
              transformation(extent={{-23,-55},{-3,-35}}, rotation=0)));
        Engine.Components.Cam cam annotation (Placement(transformation(extent=
                 {{-44,69},{-24,89}}, rotation=0)));
        Modelica.Mechanics.Rotational.Components.Inertia camshaft(
                                                       w(start=157)) annotation (Placement(
              transformation(extent={{-81,69},{-61,89}}, rotation=0)));
        Engine.Components.Valve valve1 
                                      annotation (Placement(transformation(
                extent={{35,15},{55,35}},  rotation=0)));
      equation
        connect(cam.valve_lift, valve.lift) annotation (Line(points={{-24,79},
                {-13,79},{-13,37}}));
        connect(camshaft.flange_b, cam.camshaft) annotation (Line(points={{
                -61,79},{-36.8,79}}));
        connect(upstream.tap, valve.a) annotation (Line(points={{-65,34},{-65,
                27},{-23,27}}, color={255,127,0}));
        connect(valve.b, control_volume.state) annotation (Line(points={{-13,
                17},{-13,-15}}, color={255,127,0}));
        connect(volume.y, control_volume.volume) annotation (Line(
            points={{-2,-45},{26,-45},{26,-32},{34,-32},{34,-16},{-2,-16},{-2,-15}},
            color={0,0,127},
            smooth=Smooth.None));
        connect(cam.valve_lift, valve1.lift) annotation (Line(
            points={{-24,79},{12,79},{12,35},{45,35}},
            color={0,127,0},
            smooth=Smooth.None));
        annotation (
          Window(
            x=0.17,
            y=0.23,
            width=0.6,
            height=0.6),
          Documentation(info=
                "This model exercises the valve model by opening and closing the valve (via a valvetrain cam)."),
          Diagram(graphics));
      end TestValve;

      model TestTransmission "Test the transmission and shift control"
        extends Modelica.Icons.Example;
        Transmission.FiveSpeed transmission annotation (Placement(
              transformation(extent={{-40,-40},{20,20}}, rotation=0)));
        Modelica.Mechanics.Rotational.Components.Inertia engine 
                                                     annotation (Placement(
              transformation(extent={{40,-20},{60,0}}, rotation=0)));
        Modelica.Mechanics.Rotational.Components.Inertia driveline(
                                                        J=100) annotation (Placement(
              transformation(extent={{-70,-20},{-50,0}}, rotation=0)));
        Modelica.Mechanics.Rotational.Sources.Torque engine_torque(useSupport=
             false)                                        annotation (Placement(
              transformation(extent={{50,50},{70,70}}, rotation=0)));
        Modelica.Blocks.Sources.Ramp torque_profile(height={100}) annotation (Placement(
              transformation(extent={{10,50},{30,70}}, rotation=0)));
        Modelica.Mechanics.Rotational.Components.Damper bearing(
                                                     d=10) annotation (Placement(
              transformation(
              origin={-80,-30},
              extent={{-10,-10},{10,10}},
              rotation=90)));
        Modelica.Mechanics.Rotational.Components.Fixed mount 
                                                  annotation (Placement(
              transformation(extent={{-90,-70},{-70,-50}}, rotation=0)));
        Transmission.SimpleShiftStrategy shift_strategy(tire_radius=0.35,
            up_shift_schedule={10,20,30,40}) annotation (Placement(
              transformation(extent={{-30,50},{10,90}}, rotation=0)));
      equation
        connect(driveline.flange_b, transmission.driveline) annotation (Line(
              points={{-50,-10},{-40,-10}}, color={128,128,128}));
        connect(transmission.engine, engine.flange_a) annotation (Line(points=
               {{20,-10},{40,-10}}, color={128,128,128}));
        connect(engine_torque.flange,   engine.flange_b) annotation (Line(
              points={{70,60},{80,60},{80,-10},{60,-10}}, color={128,128,128}));
        connect(torque_profile.outPort, engine_torque.inPort) annotation (Line(
              points={{31,60},{48,60}}));
        connect(driveline.flange_a, bearing.flange_b) annotation (Line(points=
               {{-70,-10},{-80,-10},{-80,-20}}, color={128,128,128}));
        connect(bearing.flange_a,mount.flange)    annotation (Line(points={{
                -80,-40},{-80,-60}}, color={128,128,128}));
        connect(shift_strategy.gear_request, transmission.gear_selector) 
          annotation (Line(points={{-10,48},{-10,23}}, color={127,0,255}));
        connect(transmission.driveline, shift_strategy.wheel) annotation (Line(
              points={{-40,-10},{-40,70},{-30,70}}, color={128,128,128}));
        annotation (
          Window(
            x=0.07,
            y=0.16,
            width=0.6,
            height=0.6),
          Documentation(info="This model tests the transmission and its control strategy by running the transmission
through a range of speeds such that all gears are engaged.
"));
      end TestTransmission;
      annotation (
        Window(
          x=0.45,
          y=0.01,
          width=0.35,
          height=0.49,
          library=1,
          autolayout=1),
        Documentation(info="The models in this package are used to verify that all the models are
working properly.  These models are not as interesting as the models
found in the 'Examples' package because they do not represent typical
or meaningful configurations of components.  Instead, these models
test the function of models by conducting virtual experiments on them
to validate the basics of their performance.
"));
    end Tests;

    package Types "Types used in vehicle modeling"
      extends Modelica.Icons.Library;
      type Degrees = Real (final unit="deg");
      type KilometersPerHour = Real (final unit="km/h");
      type RPM = Real (final unit="rev/min");
      annotation (
        Window(
          x=0.37,
          y=0.34,
          width=0.6,
          height=0.6,
          library=1,
          autolayout=1),
        Documentation(info="This package contains various types used in vehicle modeling and shared by the components and subsystem
models found throughout the 'SimpleCar' package.
"));
    end Types;

    package Vehicles "Vehicle models"
      extends Modelica.Icons.Library;
      model PassengerCar "A sample passenger car model"

        Modelica.Mechanics.Translational.Interfaces.Flange_a road annotation (Placement(
              transformation(extent={{-50,-110},{-30,-90}}, rotation=0)));
        replaceable Chassis.GenericCar chassis(vehicle_mass=1200) constrainedby
          Interfaces.Chassis annotation (Placement(transformation(extent={{
                  -80,-60},{-20,0}}, rotation=0)));
        replaceable Engine.Components.I4_Engine engine(spark_advance=20,
            burn_duration=80) constrainedby Interfaces.Engine 
                                                        annotation (Placement(
              transformation(extent={{44,-36},{64,-16}}, rotation=0)));
        Engine.GeometrySource geometry(
          bore=0.080,
          stroke=0.080,
          conrod=0.157) annotation (Placement(transformation(extent={{80,-36},
                  {100,-16}}, rotation=0)));
        Engine.Components.Reservoir intake_manifold annotation (Placement(
              transformation(extent={{20,70},{40,90}}, rotation=0)));
        Engine.Components.Reservoir exhaust_manifold annotation (Placement(
              transformation(extent={{60,0},{80,20}}, rotation=0)));
        replaceable Transmission.FiveSpeed transmission constrainedby
          Interfaces.Transmission annotation (Placement(transformation(extent=
                 {{-14,-50},{26,-10}}, rotation=0)));
        replaceable Transmission.SimpleShiftStrategy shift_strategy(tire_radius=
              0.35, up_shift_schedule={10,20,30,40}) constrainedby
          Interfaces.ShiftStrategy annotation (Placement(transformation(
                extent={{-14,10},{26,50}}, rotation=0)));
        Modelica.Blocks.Interfaces.OutPort speed annotation (Placement(
              transformation(
              origin={60,-110},
              extent={{-10,-10},{10,10}},
              rotation=270)));
        Engine.Components.Manifold intake_manifold1 annotation (Placement(
              transformation(extent={{20,40},{40,60}}, rotation=0)));
        Modelica.Blocks.Interfaces.InPort throttle annotation (Placement(
              transformation(extent={{-120,50},{-100,70}}, rotation=0)));
      //there is no more Modelica.Blocks.Interfaces.InPort in Modelica standary library
      equation
        connect(chassis.road, road) annotation (Line(points={{-62,-60},{-40,
                -60},{-40,-100}}, color={0,255,0}));
        connect(geometry.geom, engine.engine_geometry) annotation (Line(
              points={{79,-26},{65,-26}}, color={0,0,0}));
        connect(exhaust_manifold.tap, engine.exhaust) annotation (Line(points=
               {{70,0},{70,-18},{64,-18}}, color={255,127,0}));
        connect(transmission.engine, engine.crankshaft) annotation (Line(
              points={{26,-30},{44,-30}}, color={128,128,128}));
        connect(transmission.driveline, chassis.power) annotation (Line(
              points={{-14,-30},{-26,-30}}, color={128,128,128}));
        connect(shift_strategy.gear_request, transmission.gear_selector) 
          annotation (Line(points={{6,8},{6,-8}}, color={127,0,255}));
        connect(chassis.wheel, shift_strategy.wheel) annotation (Line(points=
                {{-56,-48},{-80,-48},{-80,30},{-14,30}}, color={128,128,128}));
        connect(chassis.speed, speed) annotation (Line(points={{-32,-63},{-32,
                -80},{60,-80},{60,-110}}));
        connect(intake_manifold1.manifold, engine.intake) annotation (Line(
              points={{30,40},{30,0},{44,-18}}, color={255,127,0}));
        connect(intake_manifold.tap, intake_manifold1.ambient) annotation (Line(
              points={{30,70},{30,60}}, color={255,127,0}));
        connect(intake_manifold1.throttle_angle, throttle) annotation (Line(
              points={{19,50},{10,50},{10,60},{-110,60}}));
        annotation (
          Diagram(coordinateSystem(
              preserveAspectRatio=false,
              extent={{-100,-100},{100,100}},
              grid={2,2}), graphics),
          Icon(coordinateSystem(
              preserveAspectRatio=false,
              extent={{-100,-100},{100,100}},
              grid={2,2}), graphics={
              Polygon(
                points={{-60,-38},{82,-38},{76,-20},{60,-14},{38,-10},{20,8},
                    {-28,8},{-38,-10},{-70,-18},{-70,-34},{-60,-38}},
                lineColor={0,0,0},
                fillColor={160,160,164},
                fillPattern=FillPattern.Solid),
              Ellipse(
                extent={{-50,-26},{-26,-50}},
                lineColor={0,0,255},
                pattern=LinePattern.None,
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid),
              Ellipse(
                extent={{-48,-28},{-28,-48}},
                lineColor={0,0,0},
                fillColor={128,128,128},
                fillPattern=FillPattern.Solid),
              Ellipse(
                extent={{50,-26},{74,-50}},
                lineColor={0,0,255},
                pattern=LinePattern.None,
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid),
              Ellipse(
                extent={{52,-28},{72,-48}},
                lineColor={0,0,0},
                fillColor={128,128,128},
                fillPattern=FillPattern.Solid),
              Polygon(
                points={{16,2},{30,-12},{10,-12},{-4,-12},{-4,2},{16,2}},
                lineColor={0,0,0},
                fillColor={192,192,192},
                fillPattern=FillPattern.Solid),
              Rectangle(
                extent={{-72,-28},{-62,-34}},
                lineColor={0,0,0},
                fillColor={0,0,0},
                fillPattern=FillPattern.Solid),
              Ellipse(
                extent={{56,-32},{68,-44}},
                lineColor={0,0,0},
                fillColor={192,192,192},
                fillPattern=FillPattern.Solid),
              Ellipse(
                extent={{-44,-32},{-32,-44}},
                lineColor={0,0,0},
                fillColor={192,192,192},
                fillPattern=FillPattern.Solid),
              Polygon(
                points={{-12,2},{-12,-12},{-34,-12},{-28,0},{-26,2},{-20,2},{
                    -12,2}},
                lineColor={0,0,0},
                fillColor={192,192,192},
                fillPattern=FillPattern.Solid),
              Polygon(points={{-4,6},{-4,-30},{26,-30},{34,-24},{34,-10},{18,
                    6},{-4,6}}, lineColor={128,128,128}),
              Line(points={{-40,-40},{-40,-100}}, color={127,255,0}),
              Text(extent={{-60,40},{60,20}}, textString=
                                                  "%name"),
              Line(points={{60,-52},{60,-100}})}),
          Window(
            x=0.37,
            y=0.15,
            width=0.6,
            height=0.6),
          Documentation(info="This model brings together a complete vehicle model with an engine, transmission, shift strategy and chassis.
The throttle of the vehicle is presumably controlled by an external \"driver\" model.
"));
      end PassengerCar;

      model SportsCar "Vehicle model with the characteristics of a sports car."
        Modelica.Mechanics.Translational.Interfaces.Flange_a road annotation (Placement(
              transformation(extent={{-50,-110},{-30,-90}}, rotation=0)));
        replaceable Chassis.SportsCarChassis chassis annotation (Placement(
              transformation(extent={{-80,-60},{-20,0}}, rotation=0)));
        replaceable Engine.Components.I4_Engine engine(
          spark_advance=20,
          burn_duration=60,
          evo=64,
          ivo=165,
          evc=190,
          ivc=290) annotation (Placement(transformation(extent={{44,-36},{64,
                  -16}}, rotation=0)));
        Engine.SportsCarGeometry geometry annotation (Placement(
              transformation(extent={{80,-36},{100,-16}}, rotation=0)));
        Engine.Components.Reservoir intake_ambient annotation (Placement(
              transformation(extent={{19.5365,-0.248949},{39.5365,19.7511}},
                rotation=0)));
        Engine.Components.Reservoir exhaust_manifold annotation (Placement(
              transformation(extent={{60,0},{80,20}}, rotation=0)));
        Transmission.SportsCarTransmission transmission(k_on=2, k_off=6) 
          annotation (Placement(transformation(extent={{-14,-50},{26,-10}},
                rotation=0)));
        Transmission.SimpleShiftStrategy shift_strategy(tire_radius=0.35,
            up_shift_schedule={30,60,80,95}) annotation (Placement(
              transformation(extent={{-52.4692,31.6109},{-12.4692,71.6109}},
                rotation=0)));
        Modelica.Blocks.Interfaces.RealOutput speed 
                                                 annotation (Placement(
              transformation(
              origin={60,-110},
              extent={{-10,-10},{10,10}},
              rotation=270)));
      equation
        connect(chassis.road, road) annotation (Line(points={{-62,-60},{-40,
                -60},{-40,-100}}, color={0,255,0}));
        connect(geometry.geom, engine.engine_geometry) annotation (Line(
              points={{79,-26},{65,-26}}, color={0,0,0}));
        connect(exhaust_manifold.tap, engine.exhaust) annotation (Line(points=
               {{70,0},{70,-18},{64,-18}}, color={255,127,0}));
        connect(transmission.engine, engine.crankshaft) annotation (Line(
              points={{26,-30},{44,-30}}, color={128,128,128}));
        connect(transmission.driveline, chassis.power) annotation (Line(
              points={{-14,-30},{-26,-30}}, color={128,128,128}));
        connect(chassis.wheel, shift_strategy.wheel) annotation (Line(points=
                {{-56,-48},{-80,-48},{-80,51.6109},{-52.4692,51.6109}}, color=
               {128,128,128}));
        connect(chassis.speed, speed) annotation (Line(points={{-32,-63},{-32,
                -80},{60,-80},{60,-110}}));
        connect(shift_strategy.gear_request, transmission.gear_selector) 
          annotation (Line(points={{-32.4692,29.6109},{-32.4692,0},{6,0},{6,
                -8}}));
        connect(intake_ambient.tap, engine.intake) annotation (Line(points={{
                29.5365,-0.248949},{29.5365,-18},{44,-18}}, color={255,127,0}));
        annotation (
          Diagram(coordinateSystem(
              preserveAspectRatio=false,
              extent={{-100,-100},{100,100}},
              grid={2,2}), graphics),
          Icon(coordinateSystem(
              preserveAspectRatio=false,
              extent={{-100,-100},{100,100}},
              grid={2,2}), graphics={
              Polygon(
                points={{-62,-40},{80,-40},{74,-22},{58,-16},{36,-12},{18,6},
                    {-30,6},{-40,-12},{-72,-20},{-72,-36},{-62,-40}},
                lineColor={0,0,0},
                fillColor={160,160,164},
                fillPattern=FillPattern.Solid),
              Ellipse(
                extent={{-52,-28},{-28,-52}},
                lineColor={0,0,255},
                pattern=LinePattern.None,
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid),
              Ellipse(
                extent={{-50,-30},{-30,-50}},
                lineColor={0,0,0},
                fillColor={128,128,128},
                fillPattern=FillPattern.Solid),
              Ellipse(
                extent={{48,-28},{72,-52}},
                lineColor={0,0,255},
                pattern=LinePattern.None,
                fillColor={255,255,255},
                fillPattern=FillPattern.Solid),
              Ellipse(
                extent={{50,-30},{70,-50}},
                lineColor={0,0,0},
                fillColor={128,128,128},
                fillPattern=FillPattern.Solid),
              Polygon(
                points={{16,2},{30,-12},{10,-12},{-4,-12},{-4,2},{16,2}},
                lineColor={0,0,0},
                fillColor={192,192,192},
                fillPattern=FillPattern.Solid),
              Rectangle(
                extent={{-74,-30},{-64,-36}},
                lineColor={0,0,0},
                fillColor={0,0,0},
                fillPattern=FillPattern.Solid),
              Ellipse(
                extent={{54,-34},{66,-46}},
                lineColor={0,0,0},
                fillColor={192,192,192},
                fillPattern=FillPattern.Solid),
              Ellipse(
                extent={{-46,-34},{-34,-46}},
                lineColor={0,0,0},
                fillColor={192,192,192},
                fillPattern=FillPattern.Solid),
              Polygon(
                points={{-12,2},{-12,-12},{-34,-12},{-28,0},{-26,2},{-20,2},{
                    -12,2}},
                lineColor={0,0,0},
                fillColor={192,192,192},
                fillPattern=FillPattern.Solid),
              Polygon(points={{-6,4},{-6,-32},{24,-32},{32,-26},{32,-12},{16,
                    4},{-6,4}}, lineColor={128,128,128}),
              Line(points={{-40,-40},{-40,-100}}, color={127,255,0}),
              Text(extent={{-60,40},{60,20}}, textString=
                                                  "%name"),
              Line(points={{60,-52},{60,-100}})}),
          Window(
            x=0.11,
            y=0.05,
            width=0.83,
            height=0.86));
      end SportsCar;
      annotation (
        Window(
          x=0.25,
          y=0.13,
          width=0.63,
          height=0.8,
          library=1,
          autolayout=1),
        Documentation(info="This package contains several \"complete vehicle\" models.  These are models
that combine the engine, transmission and chassis subsystems from the other
parts of the 'SimpleCar' package into a complete vehicle."),
        Icon(coordinateSystem(
            preserveAspectRatio=false,
            extent={{0,0},{994,830}},
            grid={1,1}), graphics={
            Polygon(
              points={{-73,-44},{69,-44},{63,-26},{47,-20},{25,-16},{7,2},{
                  -41,2},{-51,-16},{-83,-24},{-83,-40},{-73,-44}},
              lineColor={0,0,0},
              fillColor={160,160,164},
              fillPattern=FillPattern.Solid),
            Ellipse(
              extent={{-63,-32},{-39,-56}},
              lineColor={0,0,255},
              pattern=LinePattern.None,
              fillColor={255,255,255},
              fillPattern=FillPattern.Solid),
            Ellipse(
              extent={{37,-32},{61,-56}},
              lineColor={0,0,255},
              pattern=LinePattern.None,
              fillColor={255,255,255},
              fillPattern=FillPattern.Solid),
            Ellipse(
              extent={{-61,-34},{-41,-54}},
              lineColor={0,0,0},
              fillColor={128,128,128},
              fillPattern=FillPattern.Solid),
            Ellipse(
              extent={{-57,-38},{-45,-50}},
              lineColor={0,0,0},
              fillColor={192,192,192},
              fillPattern=FillPattern.Solid),
            Ellipse(
              extent={{39,-34},{59,-54}},
              lineColor={0,0,0},
              fillColor={128,128,128},
              fillPattern=FillPattern.Solid),
            Ellipse(
              extent={{43,-38},{55,-50}},
              lineColor={0,0,0},
              fillColor={192,192,192},
              fillPattern=FillPattern.Solid),
            Polygon(points={{-17,0},{-17,-36},{13,-36},{21,-30},{21,-16},{5,0},
                  {-17,0}}, lineColor={128,128,128}),
            Rectangle(
              extent={{-85,-34},{-75,-40}},
              lineColor={0,0,0},
              fillColor={0,0,0},
              fillPattern=FillPattern.Solid),
            Polygon(
              points={{-25,-3},{-25,-17},{-47,-17},{-41,-5},{-39,-3},{-33,-3},
                  {-25,-3}},
              lineColor={0,0,0},
              fillColor={192,192,192},
              fillPattern=FillPattern.Solid),
            Polygon(
              points={{5,-3},{19,-17},{-1,-17},{-15,-17},{-15,-3},{5,-3}},
              lineColor={0,0,0},
              fillColor={192,192,192},
              fillPattern=FillPattern.Solid)}));
    end Vehicles;
  annotation (
    Window(
      x=0.45,
      y=0.01,
      width=0.35,
      height=0.49,
      library=1,
      autolayout=1),
    Documentation(info="The \"SimpleCar\" package and all models contained in it
are copyrighted by Michael Tiller, 2001.  These models
may be copied, modified and/or resdistributed provided
that the copyright notice remains.

No warranty is provided for these models.
"));
  end Engine;
  annotation (uses(EditionGasEngine(version="1"), Modelica(version="3.1")));
end CCHPSystem;
