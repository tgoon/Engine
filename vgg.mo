within ;
model vgg

   CCHPSystem.Engine.Engine.Components.ExhaustManifold exhaustManifold(
    n=3) 
    annotation (Placement(transformation(extent={{6,22},{26,42}})));

  CCHPSystem.Engine.Engine.testSoruce testSoruce(
    mdot=1,
    P=200000,
    T=313.15) annotation (Placement(transformation(extent={{-84,44},{-64,64}})));
  CCHPSystem.Engine.Engine.testSoruce testSoruce1(
    mdot=2,
    P=300000,
    T=353.15) annotation (Placement(transformation(extent={{-86,8},{-66,28}})));
  CCHPSystem.Engine.Engine.testSoruce testSoruce2(
    mdot=1.5,
    P=150000,
    T=373.15) 
    annotation (Placement(transformation(extent={{-88,-26},{-68,-6}})));
  CCHPSystem.Engine.Engine.Sink sink 
    annotation (Placement(transformation(extent={{36,36},{56,56}})));
equation
  connect(testSoruce.gas, exhaustManifold.gas[1]) annotation (Line(
      points={{-65,55},{-22,55},{-22,22.3333},{9,22.3333}},
      color={255,0,0},
      smooth=Smooth.None));
  connect(testSoruce1.gas, exhaustManifold.gas[2]) annotation (Line(
      points={{-67,19},{9,23}},
      color={255,0,0},
      smooth=Smooth.None));
  connect(testSoruce2.gas, exhaustManifold.gas[3]) annotation (Line(
      points={{-69,-15},{-16,-15},{-16,8},{9,8},{9,23.6667}},
      color={255,0,0},
      smooth=Smooth.None));
  connect(exhaustManifold.exhust, sink.gas) annotation (Line(
      points={{25,31},{72,31},{72,47},{55,47}},
      color={255,0,0},
      smooth=Smooth.None));

  annotation (Diagram(graphics), uses(Modelica(version="3.1")));
end vgg;
