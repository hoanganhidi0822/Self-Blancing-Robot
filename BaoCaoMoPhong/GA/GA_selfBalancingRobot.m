clc;
clear all
ThongSo
rand('state',sum(100*clock));
%%%%%%%%%%%%%%%%%%%%%%%%%%%%
max_generation = 20000; %% có 200 lan chay trong quá trình
max_stall_generation = 20000; %% 50 the he giong nhau thì dung
epsilon=0.0001; %% J chuan ( neu the he nào có J<=epsilon tuc là tìm dc thông so thoa man, thì dung)

N = 30; %%so cá the trong quan the, moi cá the có moi bo PID riêng biet
npar = 9 ; %% có 9 nhiem sac the trong 1 cá the

range=[ 0   0   0   0   0   0    0   0   0   ;...
        1  1  1     1  1  1      1  1  1 ]; %% giá tri Ki kp kd nam trong khoang 
dec=[ 2 2 2 2 2 2 2 2 2  ]; %% vi tri dau cham thap phan
sig=[5 5 5 5 5 5 5 5 5 ]; %% so chu so co nghia sau dau thap phan

cross_prob = 0.8; %% muc do cá the con giong ca the me
mutate_prob = 0.2; %% he so dot bien, he so càng lon thì cá the con càng ít giong cá the me
elitism = 1; %% luon giu lai giá tri tot nhat trong khi lai tao

rho=0.08; %% trong so quyet dinh cái nào quan trong vs J hon
jmin = 10000;

par=Init(N,npar,range); %%khai tao 20 ca the cha me dau tien
Terminal=0; %khoi dong
generation = 0; %%các giá tri
stall_generation=0; %dau tien truoc khi chay GA

for pop_index=1:N,
    Kp = par(pop_index,1);
    Kd = par(pop_index,2);
    Ki = par(pop_index,3);
    Kp1 = par(pop_index,4);
    Kd1 = par(pop_index,5);
    Ki1 = par(pop_index,6);
    Kp2 = par(pop_index,7);
    Kd2 = par(pop_index,8);
    Ki2 = par(pop_index,9);
   

    sim('XeCanBangLR.slx'); %%bat dau mo phong
    if (length (e)>2000)
        J= ( (e'*e) + (e1'*e1) + (e2'*e2));
        
        jmin=J
        Kp  
        Kd
        Ki
        Kp1  
        Kd1
        Ki1
        Kp2  
        Kd2
        Ki2
  
        
        fitness(pop_index)=1/(J+eps);
    else
        J=10^100;
        fitness(pop_index)=1/(J+eps);
    end
end;
[bestfit0,bestchrom]=max(fitness);
Kp0=par(bestchrom,1); %cac ca the
Kd0=par(bestchrom,2); %dau tien
Ki0 = par(bestchrom,3);
Kp10=par(bestchrom,4); %cac ca the
Kd10=par(bestchrom,5); %dau tien
Ki10 = par(bestchrom,6);
Kp20=par(bestchrom,7); %cac ca the
Kd20=par(bestchrom,8); %dau tien
Ki20 = par(bestchrom,9);



J0=1/bestfit0 - 0.001;%do elitism= nen doi hoi phai co 1 cha me tot nhat de so sanh

while ~Terminal,%terminal se bang 1 neu chay du 200 the he hoac trong qua trinh chay co 1 the he con cai best
    generation = generation+1;%truoc moi lan chay cho hien thi
    disp(['generation #' num2str(generation) ' of maximum ' num2str(max_generation)]);
    pop=Encode_Decimal_Unsigned(par,sig,dec); %ma hoa thap phan(NST cua cac cha me)
    parent=Select_Linear_Ranking(pop,fitness,0.2,elitism,bestchrom); %sap hang cha me tuyen tinh(cha me tot nhat)
    child=Cross_Twopoint(parent,cross_prob,elitism,bestchrom);%con cai sinh ra se dc lai ghep
    pop=Mutate_Uniform(child,mutate_prob,elitism,bestchrom);%dot bien theo dang phan bo deu
    par=Decode_Decimal_Unsigned(pop,sig,dec);%giai ma ket qua ve lai dang nst

for pop_index = 1:N, %lan luot test tung ca the trong quan the
    Kp = par(pop_index,1);
    Kd = par(pop_index,2);
    Ki = par(pop_index,3);
    Kp1 = par(pop_index,4);
    Kd1 = par(pop_index,5);
    Ki1 = par(pop_index,6);
    Kp2 = par(pop_index,7);
    Kd2 = par(pop_index,8);
    Ki2 = par(pop_index,9);

    sim('XeCanBangLR.slx');%tien hanh chay mo phong de kiem tra
    if (length (e)>2000)
        J= ( (e'*e) + (e1'*e1) + (e2'*e2));
        
        jmin=J
        Kp  
        Kd
        Ki
        Kp1  
        Kd1
        Ki1
        Kp2  
        Kd2
        Ki2
        fitness(pop_index)=1/(J+eps);
    else
        J=10^10;
        fitness(pop_index)=1/(J+eps);
    end
end;
[bestfit(generation),bestchrom]=max(fitness);%ca the nao co kp1,ki1,kd1,kp2,ki2,kd2 se dc chon la ca the toi uu
if generation == max_generation %neu chay du 200 the he roi ma chua co ca the toi uu
    Terminal = 1; %thi cho terminal=1,stop GA
elseif generation>1,
    if abs(bestfit(generation)-bestfit(generation-1))<epsilon,
        stall_generation=stall_generation+1;
        if stall_generation == max_stall_generation, Terminal = 1;end %con trong qtr chay ma xuat hien ca the toi uu thi dung
    else
        stall_generation=0;
    end;
  end;

end; %While
plot(1./bestfit)

Kp = par(pop_index,1);
Kd = par(pop_index,2);
Ki = par(pop_index,3);
Kp1 = par(pop_index,4);
Kd1 = par(pop_index,5);
Ki1 = par(pop_index,6);
Kp2 = par(pop_index,7);
Kd2 = par(pop_index,8);
Ki2 = par(pop_index,9);

J = 1/bestfit(end) %ham tieu chuan tuong ung ca the con tot nhat do

sim('XeCanBangLR.slx');%tien hanh mo phong lai de kiem tra ca the con tot nhat do cho dap ung he thong nhu the nao
