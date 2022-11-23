1;
Function_File;
pkg load symbolic;      # Load the octave symbolic library
syms x1 x2;             # Define symbolic variables x1 and x1
x1_dot = -x1 + 2*x1^3 + x2;      
x2_dot = -x1 - x2;  
[x_1 x_2 jacobians eigen_values stability] = main_function(x1_dot, x2_dot);
db1 = {x_1, x_2, jacobians, eigen_values, stability};
dbout={[-1;1;0],[1;-1;0],{[5,1;-1,-1],[5,1;-1,-1],[-1,1;-1,-1]},{[4.82842712474619;-0.8284271247461901],[4.82842712474619;-0.8284271247461901],[-1 + 1i;-1 - 1i]},{"Unstable","Unstable","Stable"}};

printf("\nChecking output values with sample output")
if(cellfun(@isequal, db1, dbout))
  printf("\nOutput matched\n")
else
  printf("\nOutpput did not match\n")
endif

printf("\n\nChecking datatype for the output genereated\n")
printf("\nchecking dataype of elements of x_1\n")
for i=1:length(db1{1,1})
  try
    if(typeinfo(db1{1,1}(i,1)))=="scalar"
      printf("datatype matched\n")
    endif
  catch
    printf("datatype did not match\n")
  end_try_catch
endfor

printf("\nchecking dataype of elements of x_2\n")
for i=1:length(db1{1,2})
  try
    if(typeinfo(db1{1,2}(i,1)))=="scalar"
      printf("datatype matched\n")
    endif
  catch
    printf("datatype did not match\n")
  end_try_catch
endfor

printf("\nchecking dataype of elements of jacobians\n")
for i=1:length(db1{1,3})
  try
    for j=1:length(db1{1,3}{1,i}(1,:))
      for k=1:length(db1{1,3}{1,i}(1,:))
        if(typeinfo(db1{1,3}{1,i}(k,j)))=="scalar"
          printf("datatype matched\n")
        endif
      endfor
    endfor 
  catch
    printf("datatype did not match\n")
  end_try_catch
endfor

printf("\nchecking dataype of elements of eigen_values\n")
for i=1:length(db1{1,4})
  try
    for j=1:length(db1{1,4}{1,i}(:,1))
      if (strcmp(typeinfo(db1{1,4}{1,i}(j,1)),'scalar')||strcmp(typeinfo(db1{1,4}{1,i}(j,1)),'complex scalar'))
        printf("datatype matched\n")
      endif
    endfor 
  catch
    printf("datatype did not match\n")
  end_try_catch
endfor

printf("\nChecking datatype of elements of stability\n")
for i=1:length(db1{1,5})
  try
    if typeinfo(db1{1,5}(1,i){1,1})=="string"
      printf("datatype matched\n")
    endif
  catch
    printf("datatype did not match\n")
  end_try_catch
endfor
