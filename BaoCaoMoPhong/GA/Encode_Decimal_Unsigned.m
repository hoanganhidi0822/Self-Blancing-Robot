function pop=Encode_Decimal_Unsigned(par,sig,dec)

% ENCODE Encode parameters to base-10 chromosome strings 
%
% Input: 	par		: parameters
%  			sig		: significant figures
%			dec		: decimal point
% Output:   chrom   : chromosomes
%

% Programmed by: Huynh Thai Hoang
% Last updated : November 25, 2005

if (nargin < 3),
   error(['Too few input arguments. Use: pop=encode(par,sig,dec)']);
end;

if size(sig)~=size(dec),
   error(['Mismatch betweem SIG and DEC']);
end;

[pop_size,npar]=size(par);

% Initialize: the first parameter starts at the first digit (this is the sign digit)
% Determine the start point of the other parameters.
% It is the start of the last parameter plus
% the no. of sig. figs. plus one for sign

for pop_index = 1:pop_size,				% population pointer
  	gene_index = 1;
    for par_index = 1:npar,  			% parameter pointer                   
        
        temp(par_index)=par(pop_index,par_index)/10^dec(par_index);               
        % Encode the parameters into chromosome form 
        
		for count = 1:sig(par_index),
            temp(par_index)=temp(par_index)*10;
            pop(pop_index,gene_index)=temp(par_index)-rem(temp(par_index),1);
      	    temp(par_index)=temp(par_index)-pop(pop_index,gene_index);
            gene_index=gene_index+1;
        end         
   end % Ends "for par_index=..." loop
end    % Ends "for pop_index=..." loop

