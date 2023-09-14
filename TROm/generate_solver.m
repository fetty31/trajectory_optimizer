import casadi.*

x = SX.sym('x',2,3);
f = Function('f',{x},{sin(x)});
C = CodeGenerator('gen2.c');
C.add(f);
C.generate();