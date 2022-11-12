function [symbolic_1] = testing_octave(y)
  #printf('start testing \n');
  try
    pkg load symbolic;
    symbolic_1=2
    printf("imported symbolic \n");
  catch
    printf("failed to import symbolic \n");
    symbolic_1=0
  end_try_catch
  
  try
    pkg load control;
    symbolic_1=symbolic_1+1
    printf("imported control \n");
  catch
    printf("failed to import control \n");
    control_1='b';
  end_try_catch
  symbolic_1

