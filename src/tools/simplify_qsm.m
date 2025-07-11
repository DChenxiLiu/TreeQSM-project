% This file is part of TREEQSM.
% 
% TREEQSM is free software: you can redistribute it and/or modify
% it under the terms of the GNU General Public License as published by
% the Free Software Foundation, either version 3 of the License, or
% (at your option) any later version.
% 
% TREEQSM is distributed in the hope that it will be useful,
% but WITHOUT ANY WARRANTY; without even the implied warranty of
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
% GNU General Public License for more details.
% 
% You should have received a copy of the GNU General Public License
% along with TREEQSM.  If not, see <http://www.gnu.org/licenses/>.

function QSM = simplify_qsm(QSM,MaxOrder,SmallRadii,ReplaceIterations,Plot,Disp)

% ---------------------------------------------------------------------
% SIMPLIFY_QSM.M   Simplifies cylinder QSMs by restricting the maximum
%                       branching order, by removing thin branches, and by 
%                       replacing two concecutive cylinders with a longer cylinder
%
% Version 2.0.0
% Latest update     4 May 2022
%
% Copyright (C) 2015-2022 Pasi Raumonen
% ---------------------------------------------------------------------
%
% Inputs:
% QSM           QSM-structure, output of treeqsm.m, must contain only one model
% MaxOrder      Maximum branching order, higher order branches removed
% SmallRadii    Minimum acceptable radius for a branch at its base
% ReplaceIterations Number of iterations for replacing two concecutive
%                     cylinders inside one branch with one longer cylinder 
% Plot          If true/1, then plots the cylinder models before and
%                 after the simplification
% Disp          If Disp == 1, then display the simplication results 
%                 (the number of cylinders after each step). If 
%                 Disp == 2, then display also the treedata results for
%                 the original and simplified QSMs. If Disp == 0, then
%                 nothing is displayed.
%
% Output:
% Modified QSM      NOTICE: cylinder, branch and treedata are modified.

% Changes from version 1.1.0 to 2.0.0, 4 May 2022:
% 1) Added modification of branch and treedata structures based on the
%     modified cylinders
% 2) Added input for plotting and displaying the results
% 3) Corrected some bugs that could cause errors in some special cases 

if nargin <= 4
  Plot = 0;
  Disp = 1;
elseif nargin <= 5
  Disp = 1;
end

if Disp == 2
  inputs = QSM.rundata.inputs;
  display_treedata(QSM.treedata,inputs)
end
% Plot the cylinder model before the simplification
if Plot
  plot_cylinder_model(QSM.cylinder,'branch',1,20,1)
end

%% Maximum branching order
c = QSM.cylinder;
nc = size(c.radius,1);
if Disp >= 1
  disp([' ',num2str(nc),' cylinders originally'])
end

% Cylinders with branching order up to MaxBranchOrder
SmallOrder = c.BranchOrder <= MaxOrder; 
N = fieldnames(c);
n = max(size(N));
for i = 1:n
  c.(N{i}) = c.(N{i})(SmallOrder,:);
end

% Modify topology information
Ind = (1:1:nc)';
m = nnz(SmallOrder);
Ind(SmallOrder) = (1:1:m)';
I = c.parent > 0;
c.parent(I) = Ind(c.parent(I));
I = c.extension > 0;
c.extension(I) = Ind(c.extension(I));

if Disp == 1
  nc = nnz(SmallOrder);
  disp([' ',num2str(nc),' cylinders after branching order simplification'])
end


%% Small branches
if nargin >= 3 && SmallRadii > 0
  
  nc = size(c.radius,1);
  % Determine child branches
  BPar = QSM.branch.parent;
  nb = size(BPar,1);
  BChi = cell(nb,1);
  for i = 1:nb
    P = BPar(i);
    if P > 0
      BChi{P} = [BChi{P}; i];
    end
  end
  
  % Remove branches whose radii is too small compared to its parent
  Large = true(nc,1);
  Pass = true(nb,1);
  for i = 1:nb
    if Pass(i)
      if QSM.branch.diameter(i) < SmallRadii
        B = i;
        BC = BChi{B};
        while ~isempty(BC)
          B = [B; BC];
          BC = vertcat(BChi{BC});
        end
        Pass(B) = false;
        m = length(B);
        for k = 1:m
          Large(c.branch == B(k)) = false;
        end
      end
    end
  end
  
  % Modify topology information
  Ind = (1:1:nc)';
  m = nnz(Large);
  Ind(Large) = (1:1:m)';
  I = c.parent > 0;
  c.parent(I) = Ind(c.parent(I));
  I = c.extension > 0;
  c.extension(I) = Ind(c.extension(I));
  
  % Update/reduce cylinders
  for i = 1:n
    c.(N{i}) = c.(N{i})(Large,:);
  end
  
  if Disp >= 1
    nc = nnz(Large);
    disp([' ',num2str(nc),' cylinders after small branch simplification'])
  end
end


%% Cylinder replacing
if nargin >= 4 && ReplaceIterations > 0
  
  % Determine child cylinders
  nc = size(c.radius,1);
  CChi = cell(nc,1);
  for i = 1:nc
    P = c.parent(i);
    if P > 0
      PE = c.extension(P);
      if PE ~= i
        CChi{P} = [CChi{P}; i];
      end
    end
  end
  
  % Replace cylinders
  for j = 1:ReplaceIterations
    
    nc = size(c.radius,1);
    Ind = (1:1:nc)';
    Keep = false(nc,1);
    i = 1;
    while i <= nc
      t = 1;
      while i+t <= nc && c.branch(i+t) == c.branch(i)
        t = t+1;
      end
      Cyls = (i:1:i+t-1)';
      S = c.start(Cyls,:);
      A = c.axis(Cyls,:);
      L = c.length(Cyls);
      if t == 1 % one cylinder in the branch
        Keep(i) = true;
      elseif ceil(t/2) == floor(t/2) % even number of cylinders in the branch
        I = (1:2:t)'; % select 1., 3., 5., ...
        % Correct radii, axes and lengths
        E = S(end,:)+L(end)*A(end,:);
        S = S(I,:);
        m = length(I);
        if m > 1
          A = [S(2:end,:); E]-S(1:end,:);
        else
          A = E-S(1,:);
        end
        L = sqrt(sum(A.*A,2));
        A = [A(:,1)./L A(:,2)./L A(:,3)./L];
        cyls = Cyls(I);
        Keep(cyls) = true;
        V = pi*c.radius(Cyls).^2.*c.length(Cyls);
        J = (2:2:t)';
        V = V(I)+V(J);
        R = sqrt(V./L/pi);
        c.radius(cyls) = R;
        
      else % odd number of cylinders
        I = [1 2:2:t]'; % select 1., 2., 4., 6., ...
        % Correct radii, axes and lengths
        E = S(end,:)+L(end)*A(end,:);
        S = S(I,:);
        l = L(1);
        a = A(I,:);
        m = length(I);
        if m > 2
          a(2:end,:) = [S(3:end,:); E]-S(2:end,:);
        else
          a(2,:) = E-S(2,:);
        end
        A = a;
        L = sqrt(sum(A.*A,2));
        L(1) = l;
        A(2:end,:) = [A(2:end,1)./L(2:end) A(2:end,2)./L(2:end) A(2:end,3)./L(2:end)];
        cyls = Cyls(I);
        Keep(cyls) = true;
        V = pi*c.radius(Cyls).^2.*c.length(Cyls);
        J = (3:2:t)';
        V = V(I(2:end))+V(J);
        R = sqrt(V./L(2:end)/pi);
        c.radius(cyls(2:end)) = R;
      end
      
      if t > 1
        % Modify cylinders
        c.length(cyls) = L;
        c.axis(cyls,:) = A;
        % Correct branching/topology information
        c.PositionInBranch(cyls) = (1:1:m)';
        c.extension(cyls) = [cyls(2:end); 0];
        c.parent(cyls(2:end)) = cyls(1:end-1);
        par = c.parent(cyls(1));
        if par > 0 && ~Keep(par)
          par0 = c.parent(par);
          if Keep(par0) && c.extension(par0) == par
            c.parent(cyls(1)) = par0;
          end
        end
        
        % Correct child branches
        chi = vertcat(CChi{Cyls});
        if ~isempty(chi)
          par = c.parent(chi);
          J = Keep(par);
          par = par(~J)-1;
          c.parent(chi(~J)) = par;
          
          par = c.parent(chi);
          rp = c.radius(par);
          sp = c.start(par,:);
          ap = c.axis(par,:);
          lc = c.length(chi);
          sc = c.start(chi,:);
          ac = c.axis(chi,:);
          ec = sc+[lc.*ac(:,1) lc.*ac(:,2) lc.*ac(:,3)];
          m = length(chi);
          for k = 1:m
            [d,V,h,B] = distances_to_line(sc(k,:),ap(k,:),sp(k,:));
            V = V/d;
            sc(k,:) = sp(k,:)+rp(k)*V+B;
          end
          ac = ec-sc;
          [ac,lc] = normalize(ac);
          c.length(chi) = lc;
          c.start(chi,:) = sc;
          c.axis(chi,:) = ac;
        end
      end
      
      i = i+t;
    end
    % Change topology (parent, extension) indexes
    m = nnz(Keep);
    Ind(Keep) = (1:1:m)';
    I = c.parent > 0;
    c.parent(I) = Ind(c.parent(I));
    I = c.extension > 0;
    c.extension(I) = Ind(c.extension(I));
    
    % Update/reduce cylinders
    for i = 1:n
      c.(N{i}) = c.(N{i})(Keep,:);
    end
    
    if j < ReplaceIterations
      % Determine child cylinders
      nc = size(c.radius,1);
      CChi = cell(nc,1);
      for i = 1:nc
        P = c.parent(i);
        if P > 0
          PE = c.extension(P);
          if PE ~= i
            CChi{P} = [CChi{P}; i];
          end
        end
      end
    end
  end
  
  if Disp >= 1
    nc = size(c.radius,1);
    disp([' ',num2str(nc),' cylinders after cylinder replacements'])
  end
end
if Disp >= 1
  nc = size(c.radius,1);
  disp([' ',num2str(nc),' cylinders after all simplifications'])
end


%% Updata the QSM
% Update the branch
branch = branches(c);

% Update the treedata
inputs = QSM.rundata.inputs;
inputs.plot = 0;
% Display
if Disp == 2
  inputs.disp = 2;
else
  inputs.disp = 0;  
end
treedata = update_tree_data(QSM,c,branch,inputs);

% Update the cylinder, branch, and treedata of the QSM
QSM.cylinder = c;
QSM.branch = branch;
QSM.treedata = treedata;

% Plot the cylinder model after the simplification
if Plot
  plot_cylinder_model(QSM.cylinder,'branch',2,20,1)
end

end % End of main function


function display_treedata(treedata,inputs)
%% Generate units for displaying the treedata
Names = fieldnames(treedata);
n = size(Names,1);
Units = zeros(n,3);
m = 23;
for i = 1:n
  if ~inputs.Tria && strcmp(Names{i},'CrownVolumeAlpha')
    m = i;
  elseif inputs.Tria && strcmp(Names{i},'TriaTrunkLength')
    m = i;
  end
  if strcmp(Names{i}(1:3),'DBH')
    Units(i,:) = 'm  ';
  elseif strcmp(Names{i}(end-2:end),'ume')
    Units(i,:) = 'L  ';
  elseif strcmp(Names{i}(end-2:end),'ght')
    Units(i,:) = 'm  ';
  elseif strcmp(Names{i}(end-2:end),'gth')
    Units(i,:) = 'm  ';
  elseif strcmp(Names{i}(1:3),'vol')
    Units(i,:) = 'L  ';
  elseif strcmp(Names{i}(1:3),'len')
    Units(i,:) = 'm  ';
  elseif strcmp(Names{i}(end-2:end),'rea')
    Units(i,:) = 'm^2';
  elseif strcmp(Names{i}(1:3),'loc')
    Units(i,:) = 'm  ';
  elseif strcmp(Names{i}(end-4:end),'aConv')
    Units(i,:) = 'm^2';
  elseif strcmp(Names{i}(end-5:end),'aAlpha')
    Units(i,:) = 'm^2';
  elseif strcmp(Names{i}(end-4:end),'eConv')
    Units(i,:) = 'm^3';
  elseif strcmp(Names{i}(end-5:end),'eAlpha')
    Units(i,:) = 'm^3';
  elseif strcmp(Names{i}(end-2:end),'Ave')
    Units(i,:) = 'm  ';
  elseif strcmp(Names{i}(end-2:end),'Max')
    Units(i,:) = 'm  ';
  end
end
%% Display treedata
disp('------------')
disp('  Tree attributes before simplification:')
for i = 1:m
  v = change_precision(treedata.(Names{i}));
  if strcmp(Names{i},'DBHtri')
    disp('  -----')
    disp('  Tree attributes from triangulation:')
  end
  disp(['  ',Names{i},' = ',num2str(v),' ',Units(i,:)])
end
disp('  -----')
end