import numpy as np
from pyqubo import Binary, Constraint
from pprint import pprint
import neal
import time
import math
N = 16
K = 8
n_var = 46
a = {}
for i in range(n_var):
    if i < 100:
        a[i] = '0'
    if i < 10:
        a[i] = '00'
    a[i] = a[i] + str(i)
We = 15
x = [Binary(a[i]) for i in range(n_var)]
NodeConstraints = []
NodeConstraints.append(Constraint((x[2]+x[3]+x[8]-2*x[9]), label="En x[2]+x[3]+x[8]-2*x[9]=0"))
NodeConstraints.append(Constraint((x[4]+x[5]+x[10]-2*x[11]), label="En x[4]+x[5]+x[10]-2*x[11]=0"))
NodeConstraints.append(Constraint((x[6]+x[7]+x[12]-2*x[13]), label="En x[6]+x[7]+x[12]-2*x[13]=0"))
NodeConstraints.append(Constraint((x[1]+x[8]+x[14]-2*x[15]), label="En x[1]+x[8]+x[14]-2*x[15]=0"))
NodeConstraints.append(Constraint((x[1]+x[3]+x[16]-2*x[17]), label="En x[1]+x[3]+x[16]-2*x[17]=0"))
NodeConstraints.append(Constraint((x[10]+x[12]+x[18]-2*x[19]), label="En x[10]+x[12]+x[18]-2*x[19]=0"))
NodeConstraints.append(Constraint((x[5]+x[7]+x[20]-2*x[21]), label="En x[5]+x[7]+x[20]-2*x[21]=0"))
NodeConstraints.append(Constraint((x[14]+x[18]+x[22]-2*x[23]), label="En x[14]+x[18]+x[22]-2*x[23]=0"))
NodeConstraints.append(Constraint((x[16]+x[20]+x[24]-2*x[25]), label="En x[16]+x[20]+x[24]-2*x[25]=0"))
NodeConstraints.append(Constraint((x[8]+x[12]+x[26]-2*x[27]), label="En x[8]+x[12]+x[26]-2*x[27]=0"))
NodeConstraints.append(Constraint((x[3]+x[7]+x[28]-2*x[29]), label="En x[3]+x[7]+x[28]-2*x[29]=0"))
NodeConstraints.append(Constraint((x[0]+x[22]+x[30]-2*x[31]), label="En x[0]+x[22]+x[30]-2*x[31]=0"))
NodeConstraints.append(Constraint((x[0]+x[24]+x[32]-2*x[33]), label="En x[0]+x[24]+x[32]-2*x[33]=0"))
NodeConstraints.append(Constraint((x[0]+x[26]+x[34]-2*x[35]), label="En x[0]+x[26]+x[34]-2*x[35]=0"))
NodeConstraints.append(Constraint((x[0]+x[28]+x[36]-2*x[37]), label="En x[0]+x[28]+x[36]-2*x[37]=0"))
NodeConstraints.append(Constraint((x[0]+x[18]+x[38]-2*x[39]), label="En x[0]+x[18]+x[38]-2*x[39]=0"))
NodeConstraints.append(Constraint((x[0]+x[20]+x[40]-2*x[41]), label="En x[0]+x[20]+x[40]-2*x[41]=0"))
NodeConstraints.append(Constraint((x[0]+x[12]+x[42]-2*x[43]), label="En x[0]+x[12]+x[42]-2*x[43]=0"))
NodeConstraints.append(Constraint((x[0]+x[7]+x[44]-2*x[45]), label="En x[0]+x[7]+x[44]-2*x[45]=0"))
NodeConstraints = [c * c for c in NodeConstraints]
constraint_func = We * sum(NodeConstraints)

f = open("C0A16-8--1.in")
Numsnr = int(f.readline())
BER = []
FER = []
ENc = []

for snr in range(Numsnr):
    frame = 0
    frameerr = 0
    accerr = 0
    Nc = 0
    Ne = 0
    line1 = f.readline()
    while line1[0] != '!':
        frame = frame + 1
        line2 = f.readline()

        #y = list(map(eval, line2.split()))
        y = list(float(eval(c)) for c in line2.split())
        #y = np.array(list(map(eval, line2.split())), dtype=np.float32)

        #y = [c-0.00000001 for c in y]
        #pprint(y)
        #print(type(y[0]))
        #y[1]=y[1]+2.98023e-08;
        # pprint(y)
        # print(math.log(0.0000001, 2))
        # exit(1)
        cost_func = y[0]*(1-x[30])+y[1]*x[30]+y[2]*(1-x[32])+y[3]*x[32]+y[4]*(1-x[34])+y[5]*x[34]+y[6]*(1-x[36])+y[7]*x[36]\
        +y[8]*(1-x[38])+y[9]*x[38]+y[10]*(1-x[40])+y[11]*x[40]+y[12]*(1-x[42])+y[13]*x[42]+y[14]*(1-x[44])+y[15]*x[44]\
        +y[16]*(1-x[22])+y[17]*x[22]+y[18]*(1-x[24])+y[19]*x[24]+y[20]*(1-x[26])+y[21]*x[26]+y[22]*(1-x[28])+y[23]*x[28]\
        +y[24]*(1-x[18])+y[25]*x[18]+y[26]*(1-x[20])+y[27]*x[20]+y[28]*(1-x[12])+y[29]*x[12]+y[30]*(1-x[7])+y[31]*x[7]

        #constraints



        #QUBO form

        H = cost_func + constraint_func # minimization problem
        # compile model
        model = H.compile()
        bqm = model.to_bqm()
        # solve model
        sa = neal.SimulatedAnnealingSampler()
        sampleset = sa.sample(bqm, num_reads=50)
        decoded_samples = model.decode_sampleset(sampleset)
        best_sample = min(decoded_samples, key=lambda x: x.energy)
        #pprint(best_sample.sample)
        #print(best_sample.sample['001'])
        errs = 0
        for i in range(K):
            #print(line1[i])
            #print(best_sample.sample[a[i]])
            if line1[i] != str(best_sample.sample[a[i]]):
                errs = errs + 1
        #print(errs)
        #time.sleep(10000)
        if errs != 0:
            frameerr = frameerr + 1
            accerr = accerr + errs
        line1 = f.readline()
        #validating results
        raw_solution = best_sample.sample
        decoded_sample = model.decode_sample(raw_solution, vartype='BINARY')
        #pprint(decoded_sample.constraints(only_broken=True))
        Me = 0

        for c in decoded_sample.constraints(only_broken=True):
            if c[0] == 'E':
                Me = 1

        #print(Me)
        #print(Mc)
        #time.sleep(1)

        Ne = Ne + Me
    BER.append(accerr / frame / K)
    FER.append(frameerr / frame)
    ENc.append(Ne / frame)


f.close()
print("BER")
for snr in range(Numsnr):
    print(snr+1, BER[snr])
print("FER")
for snr in range(Numsnr):
    print(snr+1, FER[snr])
print("En constraints broken rate:")
for snr in range(Numsnr):
    print(snr+1, ENc[snr])

#print("Solution :", [best_sample.sample[key] for key in best_sample.sample.keys()])
# pprint(best_sample.sample)

# validating results
#raw_solution = best_sample.sample
#decoded_sample = model.decode_sample(raw_solution, vartype='BINARY')
#pprint(decoded_sample.constraints())
#pprint(decoded_sample.constraints(only_broken=True))
