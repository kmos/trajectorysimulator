function chi_rif_d = Calc_rif(chi_rif, chi)
%#codegen
% This block supports an embeddable subset of the MATLAB language.
% See the help menu for details. 

if chi_rif >= 0
 chi_rif2 = chi_rif - 360;
else
 chi_rif2 = chi_rif;
 chi_rif = chi_rif2 + 360;
end

D1 = chi_rif - chi;
D2 = chi_rif2 - chi;

if abs(D1) <= abs(D2)
    chi_rif_d = D1;
else
    chi_rif_d = D2;
end