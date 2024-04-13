function [raccel, laccel] = keyboard_accel_control()
  raccel = 0;
  laccel = 0;
  k=get(gcf,'CurrentCharacter');
  if k ~= '@'
    switch k
      case 'w'
        raccel = 1;
      case 's'
        raccel = -1;
      case 'e'
        raccel = 5;
      case 'd'
        raccel = -5;
      

      case 'i'
        laccel = 1;
      case 'k'
        laccel = -1;
      case 'u'
        laccel = 5;
      case 'j'
        laccel = -5;

      case 'y'
        raccel = 3;
        laccel = 3;
      case 'h'
        raccel = -3;
        laccel = -3;
    end
    clc
    raccel
    laccel

    set(gcf,'CurrentCharacter','@'); % set to a dummy character
  end
  
end