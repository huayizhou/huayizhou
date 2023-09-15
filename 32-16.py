from pyqubo import Binary, Constraint
from pprint import pprint
import neal
import time
from multiprocessing import Process, Queue


def thr1(y, line, N, K, n_var, a, We, q, index):
    x = [Binary(a[i]) for i in range(n_var)]
    NodeConstraints = []
    NodeConstraints.append(Constraint((x[0] + x[1] + x[32] - 2 * x[33]), label="En"))
    NodeConstraints.append(Constraint((x[2] + x[3] + x[34] - 2 * x[35]), label="En"))
    NodeConstraints.append(Constraint((x[4] + x[5] + x[36] - 2 * x[37]), label="En"))
    NodeConstraints.append(Constraint((x[6] + x[7] + x[38] - 2 * x[39]), label="En"))
    NodeConstraints.append(Constraint((x[8] + x[9] + x[40] - 2 * x[41]), label="En"))
    NodeConstraints.append(Constraint((x[10] + x[11] + x[42] - 2 * x[43]), label="En"))
    NodeConstraints.append(Constraint((x[12] + x[13] + x[44] - 2 * x[45]), label="En"))
    NodeConstraints.append(Constraint((x[14] + x[15] + x[46] - 2 * x[47]), label="En"))
    NodeConstraints.append(Constraint((x[16] + x[17] + x[48] - 2 * x[49]), label="En"))
    NodeConstraints.append(Constraint((x[18] + x[19] + x[50] - 2 * x[51]), label="En"))
    NodeConstraints.append(Constraint((x[20] + x[21] + x[52] - 2 * x[53]), label="En"))
    NodeConstraints.append(Constraint((x[22] + x[23] + x[54] - 2 * x[55]), label="En"))
    NodeConstraints.append(Constraint((x[24] + x[25] + x[56] - 2 * x[57]), label="En"))
    NodeConstraints.append(Constraint((x[26] + x[27] + x[58] - 2 * x[59]), label="En"))
    NodeConstraints.append(Constraint((x[28] + x[29] + x[60] - 2 * x[61]), label="En"))
    NodeConstraints.append(Constraint((x[30] + x[31] + x[62] - 2 * x[63]), label="En"))
    NodeConstraints.append(Constraint((x[32] + x[34] + x[64] - 2 * x[65]), label="En"))
    NodeConstraints.append(Constraint((x[1] + x[3] + x[66] - 2 * x[67]), label="En"))
    NodeConstraints.append(Constraint((x[36] + x[38] + x[68] - 2 * x[69]), label="En"))
    NodeConstraints.append(Constraint((x[5] + x[7] + x[70] - 2 * x[71]), label="En"))
    NodeConstraints.append(Constraint((x[40] + x[42] + x[72] - 2 * x[73]), label="En"))
    NodeConstraints.append(Constraint((x[9] + x[11] + x[74] - 2 * x[75]), label="En"))
    NodeConstraints.append(Constraint((x[44] + x[46] + x[76] - 2 * x[77]), label="En"))
    NodeConstraints.append(Constraint((x[13] + x[15] + x[78] - 2 * x[79]), label="En"))
    NodeConstraints.append(Constraint((x[48] + x[50] + x[80] - 2 * x[81]), label="En"))
    NodeConstraints.append(Constraint((x[17] + x[19] + x[82] - 2 * x[83]), label="En"))
    NodeConstraints.append(Constraint((x[52] + x[54] + x[84] - 2 * x[85]), label="En"))
    NodeConstraints.append(Constraint((x[21] + x[23] + x[86] - 2 * x[87]), label="En"))
    NodeConstraints.append(Constraint((x[56] + x[58] + x[88] - 2 * x[89]), label="En"))
    NodeConstraints.append(Constraint((x[25] + x[27] + x[90] - 2 * x[91]), label="En"))
    NodeConstraints.append(Constraint((x[60] + x[62] + x[92] - 2 * x[93]), label="En"))
    NodeConstraints.append(Constraint((x[29] + x[31] + x[94] - 2 * x[95]), label="En"))
    NodeConstraints.append(Constraint((x[64] + x[68] + x[96] - 2 * x[97]), label="En"))
    NodeConstraints.append(Constraint((x[66] + x[70] + x[98] - 2 * x[99]), label="En"))
    NodeConstraints.append(Constraint((x[34] + x[38] + x[100] - 2 * x[101]), label="En"))
    NodeConstraints.append(Constraint((x[3] + x[7] + x[102] - 2 * x[103]), label="En"))
    NodeConstraints.append(Constraint((x[72] + x[76] + x[104] - 2 * x[105]), label="En"))
    NodeConstraints.append(Constraint((x[74] + x[78] + x[106] - 2 * x[107]), label="En"))
    NodeConstraints.append(Constraint((x[42] + x[46] + x[108] - 2 * x[109]), label="En"))
    NodeConstraints.append(Constraint((x[11] + x[15] + x[110] - 2 * x[111]), label="En"))
    NodeConstraints.append(Constraint((x[80] + x[84] + x[112] - 2 * x[113]), label="En"))
    NodeConstraints.append(Constraint((x[82] + x[86] + x[114] - 2 * x[115]), label="En"))
    NodeConstraints.append(Constraint((x[50] + x[54] + x[116] - 2 * x[117]), label="En"))
    NodeConstraints.append(Constraint((x[19] + x[23] + x[118] - 2 * x[119]), label="En"))
    NodeConstraints.append(Constraint((x[88] + x[92] + x[120] - 2 * x[121]), label="En"))
    NodeConstraints.append(Constraint((x[90] + x[94] + x[122] - 2 * x[123]), label="En"))
    NodeConstraints.append(Constraint((x[58] + x[62] + x[124] - 2 * x[125]), label="En"))
    NodeConstraints.append(Constraint((x[27] + x[31] + x[126] - 2 * x[127]), label="En"))
    NodeConstraints.append(Constraint((x[96] + x[104] + x[128] - 2 * x[129]), label="En"))
    NodeConstraints.append(Constraint((x[98] + x[106] + x[130] - 2 * x[131]), label="En"))
    NodeConstraints.append(Constraint((x[100] + x[108] + x[132] - 2 * x[133]), label="En"))
    NodeConstraints.append(Constraint((x[102] + x[110] + x[134] - 2 * x[135]), label="En"))
    NodeConstraints.append(Constraint((x[68] + x[76] + x[136] - 2 * x[137]), label="En"))
    NodeConstraints.append(Constraint((x[70] + x[78] + x[138] - 2 * x[139]), label="En"))
    NodeConstraints.append(Constraint((x[38] + x[46] + x[140] - 2 * x[141]), label="En"))
    NodeConstraints.append(Constraint((x[7] + x[15] + x[142] - 2 * x[143]), label="En"))
    NodeConstraints.append(Constraint((x[112] + x[120] + x[144] - 2 * x[145]), label="En"))
    NodeConstraints.append(Constraint((x[114] + x[122] + x[146] - 2 * x[147]), label="En"))
    NodeConstraints.append(Constraint((x[116] + x[124] + x[148] - 2 * x[149]), label="En"))
    NodeConstraints.append(Constraint((x[118] + x[126] + x[150] - 2 * x[151]), label="En"))
    NodeConstraints.append(Constraint((x[84] + x[92] + x[152] - 2 * x[153]), label="En"))
    NodeConstraints.append(Constraint((x[86] + x[94] + x[154] - 2 * x[155]), label="En"))
    NodeConstraints.append(Constraint((x[54] + x[62] + x[156] - 2 * x[157]), label="En"))
    NodeConstraints.append(Constraint((x[23] + x[31] + x[158] - 2 * x[159]), label="En"))
    NodeConstraints.append(Constraint((x[128] + x[144] + x[160] - 2 * x[161]), label="En"))
    NodeConstraints.append(Constraint((x[130] + x[146] + x[162] - 2 * x[163]), label="En"))
    NodeConstraints.append(Constraint((x[132] + x[148] + x[164] - 2 * x[165]), label="En"))
    NodeConstraints.append(Constraint((x[134] + x[150] + x[166] - 2 * x[167]), label="En"))
    NodeConstraints.append(Constraint((x[136] + x[152] + x[168] - 2 * x[169]), label="En"))
    NodeConstraints.append(Constraint((x[138] + x[154] + x[170] - 2 * x[171]), label="En"))
    NodeConstraints.append(Constraint((x[140] + x[156] + x[172] - 2 * x[173]), label="En"))
    NodeConstraints.append(Constraint((x[142] + x[158] + x[174] - 2 * x[175]), label="En"))
    NodeConstraints.append(Constraint((x[104] + x[120] + x[176] - 2 * x[177]), label="En"))
    NodeConstraints.append(Constraint((x[106] + x[122] + x[178] - 2 * x[179]), label="En"))
    NodeConstraints.append(Constraint((x[108] + x[124] + x[180] - 2 * x[181]), label="En"))
    NodeConstraints.append(Constraint((x[110] + x[126] + x[182] - 2 * x[183]), label="En"))
    NodeConstraints.append(Constraint((x[76] + x[92] + x[184] - 2 * x[185]), label="En"))
    NodeConstraints.append(Constraint((x[78] + x[94] + x[186] - 2 * x[187]), label="En"))
    NodeConstraints.append(Constraint((x[46] + x[62] + x[188] - 2 * x[189]), label="En"))
    NodeConstraints.append(Constraint((x[15] + x[31] + x[190] - 2 * x[191]), label="En"))
    NodeConstraints = [c * c for c in NodeConstraints]
    constraint_func = We * sum(NodeConstraints)
    cost_func = y[0] * (1 - x[160]) + y[1] * x[160] + y[2] * (1 - x[162]) + y[3] * x[162] + y[4] * (1 - x[164]) + y[5] * \
                x[164] \
                + y[6] * (1 - x[166]) + y[7] * x[166] + y[8] * (1 - x[168]) + y[9] * x[168] + y[10] * (1 - x[170]) + y[
                    11] * x[170] \
                + y[12] * (1 - x[172]) + y[13] * x[172] + y[14] * (1 - x[174]) + y[15] * x[174] + y[16] * (1 - x[176]) + \
                y[17] * x[176] \
                + y[18] * (1 - x[178]) + y[19] * x[178] + y[20] * (1 - x[180]) + y[21] * x[180] + y[22] * (1 - x[182]) + \
                y[23] * x[182] \
                + y[24] * (1 - x[184]) + y[25] * x[184] + y[26] * (1 - x[186]) + y[27] * x[186] + y[28] * (1 - x[188]) + \
                y[29] * x[188] \
                + y[30] * (1 - x[190]) + y[31] * x[190] + y[32] * (1 - x[144]) + y[33] * x[144] + y[34] * (1 - x[146]) + \
                y[35] * x[146] \
                + y[36] * (1 - x[148]) + y[37] * x[148] + y[38] * (1 - x[150]) + y[39] * x[150] + y[40] * (1 - x[152]) + \
                y[41] * x[152] \
                + y[42] * (1 - x[154]) + y[43] * x[154] + y[44] * (1 - x[156]) + y[45] * x[156] + y[46] * (1 - x[158]) + \
                y[47] * x[158] \
                + y[48] * (1 - x[120]) + y[49] * x[120] + y[50] * (1 - x[122]) + y[51] * x[122] + y[52] * (1 - x[124]) + \
                y[53] * x[124] \
                + y[54] * (1 - x[126]) + y[55] * x[126] + y[56] * (1 - x[92]) + y[57] * x[92] + y[58] * (1 - x[94]) + y[
                    59] * x[94] \
                + y[60] * (1 - x[62]) + y[61] * x[62] + y[62] * (1 - x[31]) + y[63] * x[31] + 100 * (
                            x[0] + x[1] + x[2] + x[3] + x[4] + x[5] + x[6] + x[8] + x[9] + x[10] + x[12] + x[16] + x[
                        17] + x[18] + x[20] + x[24])
    H = cost_func + constraint_func
    model = H.compile()
    bqm = model.to_bqm()
    sa = neal.SimulatedAnnealingSampler()
    sampleset = sa.sample(bqm, num_reads=300)
    decoded_samples = model.decode_sampleset(sampleset)
    best_sample = min(decoded_samples, key=lambda x: x.energy)
    result = [0, 0]
    for i in range(K):
        if line[i] != str(best_sample.sample[a[index[i]]]):
            result[0] = result[0] + 1
    raw_solution = best_sample.sample
    decoded_sample = model.decode_sample(raw_solution, vartype='BINARY')
    for c in decoded_sample.constraints(only_broken=True):
        if c[0] == 'E':
            result[1] = 1
    q.put(result)


def thr2(y, line, N, K, n_var, a, We, q, index):
    x = [Binary(a[i]) for i in range(n_var)]
    NodeConstraints = []
    NodeConstraints.append(Constraint((x[0] + x[1] + x[32] - 2 * x[33]), label="En"))
    NodeConstraints.append(Constraint((x[2] + x[3] + x[34] - 2 * x[35]), label="En"))
    NodeConstraints.append(Constraint((x[4] + x[5] + x[36] - 2 * x[37]), label="En"))
    NodeConstraints.append(Constraint((x[6] + x[7] + x[38] - 2 * x[39]), label="En"))
    NodeConstraints.append(Constraint((x[8] + x[9] + x[40] - 2 * x[41]), label="En"))
    NodeConstraints.append(Constraint((x[10] + x[11] + x[42] - 2 * x[43]), label="En"))
    NodeConstraints.append(Constraint((x[12] + x[13] + x[44] - 2 * x[45]), label="En"))
    NodeConstraints.append(Constraint((x[14] + x[15] + x[46] - 2 * x[47]), label="En"))
    NodeConstraints.append(Constraint((x[16] + x[17] + x[48] - 2 * x[49]), label="En"))
    NodeConstraints.append(Constraint((x[18] + x[19] + x[50] - 2 * x[51]), label="En"))
    NodeConstraints.append(Constraint((x[20] + x[21] + x[52] - 2 * x[53]), label="En"))
    NodeConstraints.append(Constraint((x[22] + x[23] + x[54] - 2 * x[55]), label="En"))
    NodeConstraints.append(Constraint((x[24] + x[25] + x[56] - 2 * x[57]), label="En"))
    NodeConstraints.append(Constraint((x[26] + x[27] + x[58] - 2 * x[59]), label="En"))
    NodeConstraints.append(Constraint((x[28] + x[29] + x[60] - 2 * x[61]), label="En"))
    NodeConstraints.append(Constraint((x[30] + x[31] + x[62] - 2 * x[63]), label="En"))
    NodeConstraints.append(Constraint((x[32] + x[34] + x[64] - 2 * x[65]), label="En"))
    NodeConstraints.append(Constraint((x[1] + x[3] + x[66] - 2 * x[67]), label="En"))
    NodeConstraints.append(Constraint((x[36] + x[38] + x[68] - 2 * x[69]), label="En"))
    NodeConstraints.append(Constraint((x[5] + x[7] + x[70] - 2 * x[71]), label="En"))
    NodeConstraints.append(Constraint((x[40] + x[42] + x[72] - 2 * x[73]), label="En"))
    NodeConstraints.append(Constraint((x[9] + x[11] + x[74] - 2 * x[75]), label="En"))
    NodeConstraints.append(Constraint((x[44] + x[46] + x[76] - 2 * x[77]), label="En"))
    NodeConstraints.append(Constraint((x[13] + x[15] + x[78] - 2 * x[79]), label="En"))
    NodeConstraints.append(Constraint((x[48] + x[50] + x[80] - 2 * x[81]), label="En"))
    NodeConstraints.append(Constraint((x[17] + x[19] + x[82] - 2 * x[83]), label="En"))
    NodeConstraints.append(Constraint((x[52] + x[54] + x[84] - 2 * x[85]), label="En"))
    NodeConstraints.append(Constraint((x[21] + x[23] + x[86] - 2 * x[87]), label="En"))
    NodeConstraints.append(Constraint((x[56] + x[58] + x[88] - 2 * x[89]), label="En"))
    NodeConstraints.append(Constraint((x[25] + x[27] + x[90] - 2 * x[91]), label="En"))
    NodeConstraints.append(Constraint((x[60] + x[62] + x[92] - 2 * x[93]), label="En"))
    NodeConstraints.append(Constraint((x[29] + x[31] + x[94] - 2 * x[95]), label="En"))
    NodeConstraints.append(Constraint((x[64] + x[68] + x[96] - 2 * x[97]), label="En"))
    NodeConstraints.append(Constraint((x[66] + x[70] + x[98] - 2 * x[99]), label="En"))
    NodeConstraints.append(Constraint((x[34] + x[38] + x[100] - 2 * x[101]), label="En"))
    NodeConstraints.append(Constraint((x[3] + x[7] + x[102] - 2 * x[103]), label="En"))
    NodeConstraints.append(Constraint((x[72] + x[76] + x[104] - 2 * x[105]), label="En"))
    NodeConstraints.append(Constraint((x[74] + x[78] + x[106] - 2 * x[107]), label="En"))
    NodeConstraints.append(Constraint((x[42] + x[46] + x[108] - 2 * x[109]), label="En"))
    NodeConstraints.append(Constraint((x[11] + x[15] + x[110] - 2 * x[111]), label="En"))
    NodeConstraints.append(Constraint((x[80] + x[84] + x[112] - 2 * x[113]), label="En"))
    NodeConstraints.append(Constraint((x[82] + x[86] + x[114] - 2 * x[115]), label="En"))
    NodeConstraints.append(Constraint((x[50] + x[54] + x[116] - 2 * x[117]), label="En"))
    NodeConstraints.append(Constraint((x[19] + x[23] + x[118] - 2 * x[119]), label="En"))
    NodeConstraints.append(Constraint((x[88] + x[92] + x[120] - 2 * x[121]), label="En"))
    NodeConstraints.append(Constraint((x[90] + x[94] + x[122] - 2 * x[123]), label="En"))
    NodeConstraints.append(Constraint((x[58] + x[62] + x[124] - 2 * x[125]), label="En"))
    NodeConstraints.append(Constraint((x[27] + x[31] + x[126] - 2 * x[127]), label="En"))
    NodeConstraints.append(Constraint((x[96] + x[104] + x[128] - 2 * x[129]), label="En"))
    NodeConstraints.append(Constraint((x[98] + x[106] + x[130] - 2 * x[131]), label="En"))
    NodeConstraints.append(Constraint((x[100] + x[108] + x[132] - 2 * x[133]), label="En"))
    NodeConstraints.append(Constraint((x[102] + x[110] + x[134] - 2 * x[135]), label="En"))
    NodeConstraints.append(Constraint((x[68] + x[76] + x[136] - 2 * x[137]), label="En"))
    NodeConstraints.append(Constraint((x[70] + x[78] + x[138] - 2 * x[139]), label="En"))
    NodeConstraints.append(Constraint((x[38] + x[46] + x[140] - 2 * x[141]), label="En"))
    NodeConstraints.append(Constraint((x[7] + x[15] + x[142] - 2 * x[143]), label="En"))
    NodeConstraints.append(Constraint((x[112] + x[120] + x[144] - 2 * x[145]), label="En"))
    NodeConstraints.append(Constraint((x[114] + x[122] + x[146] - 2 * x[147]), label="En"))
    NodeConstraints.append(Constraint((x[116] + x[124] + x[148] - 2 * x[149]), label="En"))
    NodeConstraints.append(Constraint((x[118] + x[126] + x[150] - 2 * x[151]), label="En"))
    NodeConstraints.append(Constraint((x[84] + x[92] + x[152] - 2 * x[153]), label="En"))
    NodeConstraints.append(Constraint((x[86] + x[94] + x[154] - 2 * x[155]), label="En"))
    NodeConstraints.append(Constraint((x[54] + x[62] + x[156] - 2 * x[157]), label="En"))
    NodeConstraints.append(Constraint((x[23] + x[31] + x[158] - 2 * x[159]), label="En"))
    NodeConstraints.append(Constraint((x[128] + x[144] + x[160] - 2 * x[161]), label="En"))
    NodeConstraints.append(Constraint((x[130] + x[146] + x[162] - 2 * x[163]), label="En"))
    NodeConstraints.append(Constraint((x[132] + x[148] + x[164] - 2 * x[165]), label="En"))
    NodeConstraints.append(Constraint((x[134] + x[150] + x[166] - 2 * x[167]), label="En"))
    NodeConstraints.append(Constraint((x[136] + x[152] + x[168] - 2 * x[169]), label="En"))
    NodeConstraints.append(Constraint((x[138] + x[154] + x[170] - 2 * x[171]), label="En"))
    NodeConstraints.append(Constraint((x[140] + x[156] + x[172] - 2 * x[173]), label="En"))
    NodeConstraints.append(Constraint((x[142] + x[158] + x[174] - 2 * x[175]), label="En"))
    NodeConstraints.append(Constraint((x[104] + x[120] + x[176] - 2 * x[177]), label="En"))
    NodeConstraints.append(Constraint((x[106] + x[122] + x[178] - 2 * x[179]), label="En"))
    NodeConstraints.append(Constraint((x[108] + x[124] + x[180] - 2 * x[181]), label="En"))
    NodeConstraints.append(Constraint((x[110] + x[126] + x[182] - 2 * x[183]), label="En"))
    NodeConstraints.append(Constraint((x[76] + x[92] + x[184] - 2 * x[185]), label="En"))
    NodeConstraints.append(Constraint((x[78] + x[94] + x[186] - 2 * x[187]), label="En"))
    NodeConstraints.append(Constraint((x[46] + x[62] + x[188] - 2 * x[189]), label="En"))
    NodeConstraints.append(Constraint((x[15] + x[31] + x[190] - 2 * x[191]), label="En"))
    NodeConstraints = [c * c for c in NodeConstraints]
    constraint_func = We * sum(NodeConstraints)
    cost_func = y[0] * (1 - x[160]) + y[1] * x[160] + y[2] * (1 - x[162]) + y[3] * x[162] + y[4] * (1 - x[164]) + y[5] * \
                x[164] \
                + y[6] * (1 - x[166]) + y[7] * x[166] + y[8] * (1 - x[168]) + y[9] * x[168] + y[10] * (1 - x[170]) + y[
                    11] * x[170] \
                + y[12] * (1 - x[172]) + y[13] * x[172] + y[14] * (1 - x[174]) + y[15] * x[174] + y[16] * (1 - x[176]) + \
                y[17] * x[176] \
                + y[18] * (1 - x[178]) + y[19] * x[178] + y[20] * (1 - x[180]) + y[21] * x[180] + y[22] * (1 - x[182]) + \
                y[23] * x[182] \
                + y[24] * (1 - x[184]) + y[25] * x[184] + y[26] * (1 - x[186]) + y[27] * x[186] + y[28] * (1 - x[188]) + \
                y[29] * x[188] \
                + y[30] * (1 - x[190]) + y[31] * x[190] + y[32] * (1 - x[144]) + y[33] * x[144] + y[34] * (1 - x[146]) + \
                y[35] * x[146] \
                + y[36] * (1 - x[148]) + y[37] * x[148] + y[38] * (1 - x[150]) + y[39] * x[150] + y[40] * (1 - x[152]) + \
                y[41] * x[152] \
                + y[42] * (1 - x[154]) + y[43] * x[154] + y[44] * (1 - x[156]) + y[45] * x[156] + y[46] * (1 - x[158]) + \
                y[47] * x[158] \
                + y[48] * (1 - x[120]) + y[49] * x[120] + y[50] * (1 - x[122]) + y[51] * x[122] + y[52] * (1 - x[124]) + \
                y[53] * x[124] \
                + y[54] * (1 - x[126]) + y[55] * x[126] + y[56] * (1 - x[92]) + y[57] * x[92] + y[58] * (1 - x[94]) + y[
                    59] * x[94] \
                + y[60] * (1 - x[62]) + y[61] * x[62] + y[62] * (1 - x[31]) + y[63] * x[31] + 100 * (
                            x[0] + x[1] + x[2] + x[3] + x[4] + x[5] + x[6] + x[8] + x[9] + x[10] + x[12] + x[16] + x[
                        17] + x[18] + x[20] + x[24])
    H = cost_func + constraint_func
    model = H.compile()
    bqm = model.to_bqm()
    sa = neal.SimulatedAnnealingSampler()
    sampleset = sa.sample(bqm, num_reads=300)
    decoded_samples = model.decode_sampleset(sampleset)
    best_sample = min(decoded_samples, key=lambda x: x.energy)
    result = [0, 0]
    for i in range(K):
        if line[i] != str(best_sample.sample[a[index[i]]]):
            result[0] = result[0] + 1
    raw_solution = best_sample.sample
    decoded_sample = model.decode_sample(raw_solution, vartype='BINARY')
    for c in decoded_sample.constraints(only_broken=True):
        if c[0] == 'E':
            result[1] = 1
    q.put(result)


def thr3(y, line, N, K, n_var, a, We, q, index):
    x = [Binary(a[i]) for i in range(n_var)]
    NodeConstraints = []
    NodeConstraints.append(Constraint((x[0] + x[1] + x[32] - 2 * x[33]), label="En"))
    NodeConstraints.append(Constraint((x[2] + x[3] + x[34] - 2 * x[35]), label="En"))
    NodeConstraints.append(Constraint((x[4] + x[5] + x[36] - 2 * x[37]), label="En"))
    NodeConstraints.append(Constraint((x[6] + x[7] + x[38] - 2 * x[39]), label="En"))
    NodeConstraints.append(Constraint((x[8] + x[9] + x[40] - 2 * x[41]), label="En"))
    NodeConstraints.append(Constraint((x[10] + x[11] + x[42] - 2 * x[43]), label="En"))
    NodeConstraints.append(Constraint((x[12] + x[13] + x[44] - 2 * x[45]), label="En"))
    NodeConstraints.append(Constraint((x[14] + x[15] + x[46] - 2 * x[47]), label="En"))
    NodeConstraints.append(Constraint((x[16] + x[17] + x[48] - 2 * x[49]), label="En"))
    NodeConstraints.append(Constraint((x[18] + x[19] + x[50] - 2 * x[51]), label="En"))
    NodeConstraints.append(Constraint((x[20] + x[21] + x[52] - 2 * x[53]), label="En"))
    NodeConstraints.append(Constraint((x[22] + x[23] + x[54] - 2 * x[55]), label="En"))
    NodeConstraints.append(Constraint((x[24] + x[25] + x[56] - 2 * x[57]), label="En"))
    NodeConstraints.append(Constraint((x[26] + x[27] + x[58] - 2 * x[59]), label="En"))
    NodeConstraints.append(Constraint((x[28] + x[29] + x[60] - 2 * x[61]), label="En"))
    NodeConstraints.append(Constraint((x[30] + x[31] + x[62] - 2 * x[63]), label="En"))
    NodeConstraints.append(Constraint((x[32] + x[34] + x[64] - 2 * x[65]), label="En"))
    NodeConstraints.append(Constraint((x[1] + x[3] + x[66] - 2 * x[67]), label="En"))
    NodeConstraints.append(Constraint((x[36] + x[38] + x[68] - 2 * x[69]), label="En"))
    NodeConstraints.append(Constraint((x[5] + x[7] + x[70] - 2 * x[71]), label="En"))
    NodeConstraints.append(Constraint((x[40] + x[42] + x[72] - 2 * x[73]), label="En"))
    NodeConstraints.append(Constraint((x[9] + x[11] + x[74] - 2 * x[75]), label="En"))
    NodeConstraints.append(Constraint((x[44] + x[46] + x[76] - 2 * x[77]), label="En"))
    NodeConstraints.append(Constraint((x[13] + x[15] + x[78] - 2 * x[79]), label="En"))
    NodeConstraints.append(Constraint((x[48] + x[50] + x[80] - 2 * x[81]), label="En"))
    NodeConstraints.append(Constraint((x[17] + x[19] + x[82] - 2 * x[83]), label="En"))
    NodeConstraints.append(Constraint((x[52] + x[54] + x[84] - 2 * x[85]), label="En"))
    NodeConstraints.append(Constraint((x[21] + x[23] + x[86] - 2 * x[87]), label="En"))
    NodeConstraints.append(Constraint((x[56] + x[58] + x[88] - 2 * x[89]), label="En"))
    NodeConstraints.append(Constraint((x[25] + x[27] + x[90] - 2 * x[91]), label="En"))
    NodeConstraints.append(Constraint((x[60] + x[62] + x[92] - 2 * x[93]), label="En"))
    NodeConstraints.append(Constraint((x[29] + x[31] + x[94] - 2 * x[95]), label="En"))
    NodeConstraints.append(Constraint((x[64] + x[68] + x[96] - 2 * x[97]), label="En"))
    NodeConstraints.append(Constraint((x[66] + x[70] + x[98] - 2 * x[99]), label="En"))
    NodeConstraints.append(Constraint((x[34] + x[38] + x[100] - 2 * x[101]), label="En"))
    NodeConstraints.append(Constraint((x[3] + x[7] + x[102] - 2 * x[103]), label="En"))
    NodeConstraints.append(Constraint((x[72] + x[76] + x[104] - 2 * x[105]), label="En"))
    NodeConstraints.append(Constraint((x[74] + x[78] + x[106] - 2 * x[107]), label="En"))
    NodeConstraints.append(Constraint((x[42] + x[46] + x[108] - 2 * x[109]), label="En"))
    NodeConstraints.append(Constraint((x[11] + x[15] + x[110] - 2 * x[111]), label="En"))
    NodeConstraints.append(Constraint((x[80] + x[84] + x[112] - 2 * x[113]), label="En"))
    NodeConstraints.append(Constraint((x[82] + x[86] + x[114] - 2 * x[115]), label="En"))
    NodeConstraints.append(Constraint((x[50] + x[54] + x[116] - 2 * x[117]), label="En"))
    NodeConstraints.append(Constraint((x[19] + x[23] + x[118] - 2 * x[119]), label="En"))
    NodeConstraints.append(Constraint((x[88] + x[92] + x[120] - 2 * x[121]), label="En"))
    NodeConstraints.append(Constraint((x[90] + x[94] + x[122] - 2 * x[123]), label="En"))
    NodeConstraints.append(Constraint((x[58] + x[62] + x[124] - 2 * x[125]), label="En"))
    NodeConstraints.append(Constraint((x[27] + x[31] + x[126] - 2 * x[127]), label="En"))
    NodeConstraints.append(Constraint((x[96] + x[104] + x[128] - 2 * x[129]), label="En"))
    NodeConstraints.append(Constraint((x[98] + x[106] + x[130] - 2 * x[131]), label="En"))
    NodeConstraints.append(Constraint((x[100] + x[108] + x[132] - 2 * x[133]), label="En"))
    NodeConstraints.append(Constraint((x[102] + x[110] + x[134] - 2 * x[135]), label="En"))
    NodeConstraints.append(Constraint((x[68] + x[76] + x[136] - 2 * x[137]), label="En"))
    NodeConstraints.append(Constraint((x[70] + x[78] + x[138] - 2 * x[139]), label="En"))
    NodeConstraints.append(Constraint((x[38] + x[46] + x[140] - 2 * x[141]), label="En"))
    NodeConstraints.append(Constraint((x[7] + x[15] + x[142] - 2 * x[143]), label="En"))
    NodeConstraints.append(Constraint((x[112] + x[120] + x[144] - 2 * x[145]), label="En"))
    NodeConstraints.append(Constraint((x[114] + x[122] + x[146] - 2 * x[147]), label="En"))
    NodeConstraints.append(Constraint((x[116] + x[124] + x[148] - 2 * x[149]), label="En"))
    NodeConstraints.append(Constraint((x[118] + x[126] + x[150] - 2 * x[151]), label="En"))
    NodeConstraints.append(Constraint((x[84] + x[92] + x[152] - 2 * x[153]), label="En"))
    NodeConstraints.append(Constraint((x[86] + x[94] + x[154] - 2 * x[155]), label="En"))
    NodeConstraints.append(Constraint((x[54] + x[62] + x[156] - 2 * x[157]), label="En"))
    NodeConstraints.append(Constraint((x[23] + x[31] + x[158] - 2 * x[159]), label="En"))
    NodeConstraints.append(Constraint((x[128] + x[144] + x[160] - 2 * x[161]), label="En"))
    NodeConstraints.append(Constraint((x[130] + x[146] + x[162] - 2 * x[163]), label="En"))
    NodeConstraints.append(Constraint((x[132] + x[148] + x[164] - 2 * x[165]), label="En"))
    NodeConstraints.append(Constraint((x[134] + x[150] + x[166] - 2 * x[167]), label="En"))
    NodeConstraints.append(Constraint((x[136] + x[152] + x[168] - 2 * x[169]), label="En"))
    NodeConstraints.append(Constraint((x[138] + x[154] + x[170] - 2 * x[171]), label="En"))
    NodeConstraints.append(Constraint((x[140] + x[156] + x[172] - 2 * x[173]), label="En"))
    NodeConstraints.append(Constraint((x[142] + x[158] + x[174] - 2 * x[175]), label="En"))
    NodeConstraints.append(Constraint((x[104] + x[120] + x[176] - 2 * x[177]), label="En"))
    NodeConstraints.append(Constraint((x[106] + x[122] + x[178] - 2 * x[179]), label="En"))
    NodeConstraints.append(Constraint((x[108] + x[124] + x[180] - 2 * x[181]), label="En"))
    NodeConstraints.append(Constraint((x[110] + x[126] + x[182] - 2 * x[183]), label="En"))
    NodeConstraints.append(Constraint((x[76] + x[92] + x[184] - 2 * x[185]), label="En"))
    NodeConstraints.append(Constraint((x[78] + x[94] + x[186] - 2 * x[187]), label="En"))
    NodeConstraints.append(Constraint((x[46] + x[62] + x[188] - 2 * x[189]), label="En"))
    NodeConstraints.append(Constraint((x[15] + x[31] + x[190] - 2 * x[191]), label="En"))
    NodeConstraints = [c * c for c in NodeConstraints]
    constraint_func = We * sum(NodeConstraints)
    cost_func = y[0] * (1 - x[160]) + y[1] * x[160] + y[2] * (1 - x[162]) + y[3] * x[162] + y[4] * (1 - x[164]) + y[5] * \
                x[164] \
                + y[6] * (1 - x[166]) + y[7] * x[166] + y[8] * (1 - x[168]) + y[9] * x[168] + y[10] * (1 - x[170]) + y[
                    11] * x[170] \
                + y[12] * (1 - x[172]) + y[13] * x[172] + y[14] * (1 - x[174]) + y[15] * x[174] + y[16] * (1 - x[176]) + \
                y[17] * x[176] \
                + y[18] * (1 - x[178]) + y[19] * x[178] + y[20] * (1 - x[180]) + y[21] * x[180] + y[22] * (1 - x[182]) + \
                y[23] * x[182] \
                + y[24] * (1 - x[184]) + y[25] * x[184] + y[26] * (1 - x[186]) + y[27] * x[186] + y[28] * (1 - x[188]) + \
                y[29] * x[188] \
                + y[30] * (1 - x[190]) + y[31] * x[190] + y[32] * (1 - x[144]) + y[33] * x[144] + y[34] * (1 - x[146]) + \
                y[35] * x[146] \
                + y[36] * (1 - x[148]) + y[37] * x[148] + y[38] * (1 - x[150]) + y[39] * x[150] + y[40] * (1 - x[152]) + \
                y[41] * x[152] \
                + y[42] * (1 - x[154]) + y[43] * x[154] + y[44] * (1 - x[156]) + y[45] * x[156] + y[46] * (1 - x[158]) + \
                y[47] * x[158] \
                + y[48] * (1 - x[120]) + y[49] * x[120] + y[50] * (1 - x[122]) + y[51] * x[122] + y[52] * (1 - x[124]) + \
                y[53] * x[124] \
                + y[54] * (1 - x[126]) + y[55] * x[126] + y[56] * (1 - x[92]) + y[57] * x[92] + y[58] * (1 - x[94]) + y[
                    59] * x[94] \
                + y[60] * (1 - x[62]) + y[61] * x[62] + y[62] * (1 - x[31]) + y[63] * x[31] + 100 * (
                            x[0] + x[1] + x[2] + x[3] + x[4] + x[5] + x[6] + x[8] + x[9] + x[10] + x[12] + x[16] + x[
                        17] + x[18] + x[20] + x[24])
    H = cost_func + constraint_func
    model = H.compile()
    bqm = model.to_bqm()
    sa = neal.SimulatedAnnealingSampler()
    sampleset = sa.sample(bqm, num_reads=300)
    decoded_samples = model.decode_sampleset(sampleset)
    best_sample = min(decoded_samples, key=lambda x: x.energy)
    result = [0, 0]
    for i in range(K):
        if line[i] != str(best_sample.sample[a[index[i]]]):
            result[0] = result[0] + 1
    raw_solution = best_sample.sample
    decoded_sample = model.decode_sample(raw_solution, vartype='BINARY')
    for c in decoded_sample.constraints(only_broken=True):
        if c[0] == 'E':
            result[1] = 1
    q.put(result)


def thr4(y, line, N, K, n_var, a, We, q, index):
    x = [Binary(a[i]) for i in range(n_var)]
    NodeConstraints = []
    NodeConstraints.append(Constraint((x[0] + x[1] + x[32] - 2 * x[33]), label="En"))
    NodeConstraints.append(Constraint((x[2] + x[3] + x[34] - 2 * x[35]), label="En"))
    NodeConstraints.append(Constraint((x[4] + x[5] + x[36] - 2 * x[37]), label="En"))
    NodeConstraints.append(Constraint((x[6] + x[7] + x[38] - 2 * x[39]), label="En"))
    NodeConstraints.append(Constraint((x[8] + x[9] + x[40] - 2 * x[41]), label="En"))
    NodeConstraints.append(Constraint((x[10] + x[11] + x[42] - 2 * x[43]), label="En"))
    NodeConstraints.append(Constraint((x[12] + x[13] + x[44] - 2 * x[45]), label="En"))
    NodeConstraints.append(Constraint((x[14] + x[15] + x[46] - 2 * x[47]), label="En"))
    NodeConstraints.append(Constraint((x[16] + x[17] + x[48] - 2 * x[49]), label="En"))
    NodeConstraints.append(Constraint((x[18] + x[19] + x[50] - 2 * x[51]), label="En"))
    NodeConstraints.append(Constraint((x[20] + x[21] + x[52] - 2 * x[53]), label="En"))
    NodeConstraints.append(Constraint((x[22] + x[23] + x[54] - 2 * x[55]), label="En"))
    NodeConstraints.append(Constraint((x[24] + x[25] + x[56] - 2 * x[57]), label="En"))
    NodeConstraints.append(Constraint((x[26] + x[27] + x[58] - 2 * x[59]), label="En"))
    NodeConstraints.append(Constraint((x[28] + x[29] + x[60] - 2 * x[61]), label="En"))
    NodeConstraints.append(Constraint((x[30] + x[31] + x[62] - 2 * x[63]), label="En"))
    NodeConstraints.append(Constraint((x[32] + x[34] + x[64] - 2 * x[65]), label="En"))
    NodeConstraints.append(Constraint((x[1] + x[3] + x[66] - 2 * x[67]), label="En"))
    NodeConstraints.append(Constraint((x[36] + x[38] + x[68] - 2 * x[69]), label="En"))
    NodeConstraints.append(Constraint((x[5] + x[7] + x[70] - 2 * x[71]), label="En"))
    NodeConstraints.append(Constraint((x[40] + x[42] + x[72] - 2 * x[73]), label="En"))
    NodeConstraints.append(Constraint((x[9] + x[11] + x[74] - 2 * x[75]), label="En"))
    NodeConstraints.append(Constraint((x[44] + x[46] + x[76] - 2 * x[77]), label="En"))
    NodeConstraints.append(Constraint((x[13] + x[15] + x[78] - 2 * x[79]), label="En"))
    NodeConstraints.append(Constraint((x[48] + x[50] + x[80] - 2 * x[81]), label="En"))
    NodeConstraints.append(Constraint((x[17] + x[19] + x[82] - 2 * x[83]), label="En"))
    NodeConstraints.append(Constraint((x[52] + x[54] + x[84] - 2 * x[85]), label="En"))
    NodeConstraints.append(Constraint((x[21] + x[23] + x[86] - 2 * x[87]), label="En"))
    NodeConstraints.append(Constraint((x[56] + x[58] + x[88] - 2 * x[89]), label="En"))
    NodeConstraints.append(Constraint((x[25] + x[27] + x[90] - 2 * x[91]), label="En"))
    NodeConstraints.append(Constraint((x[60] + x[62] + x[92] - 2 * x[93]), label="En"))
    NodeConstraints.append(Constraint((x[29] + x[31] + x[94] - 2 * x[95]), label="En"))
    NodeConstraints.append(Constraint((x[64] + x[68] + x[96] - 2 * x[97]), label="En"))
    NodeConstraints.append(Constraint((x[66] + x[70] + x[98] - 2 * x[99]), label="En"))
    NodeConstraints.append(Constraint((x[34] + x[38] + x[100] - 2 * x[101]), label="En"))
    NodeConstraints.append(Constraint((x[3] + x[7] + x[102] - 2 * x[103]), label="En"))
    NodeConstraints.append(Constraint((x[72] + x[76] + x[104] - 2 * x[105]), label="En"))
    NodeConstraints.append(Constraint((x[74] + x[78] + x[106] - 2 * x[107]), label="En"))
    NodeConstraints.append(Constraint((x[42] + x[46] + x[108] - 2 * x[109]), label="En"))
    NodeConstraints.append(Constraint((x[11] + x[15] + x[110] - 2 * x[111]), label="En"))
    NodeConstraints.append(Constraint((x[80] + x[84] + x[112] - 2 * x[113]), label="En"))
    NodeConstraints.append(Constraint((x[82] + x[86] + x[114] - 2 * x[115]), label="En"))
    NodeConstraints.append(Constraint((x[50] + x[54] + x[116] - 2 * x[117]), label="En"))
    NodeConstraints.append(Constraint((x[19] + x[23] + x[118] - 2 * x[119]), label="En"))
    NodeConstraints.append(Constraint((x[88] + x[92] + x[120] - 2 * x[121]), label="En"))
    NodeConstraints.append(Constraint((x[90] + x[94] + x[122] - 2 * x[123]), label="En"))
    NodeConstraints.append(Constraint((x[58] + x[62] + x[124] - 2 * x[125]), label="En"))
    NodeConstraints.append(Constraint((x[27] + x[31] + x[126] - 2 * x[127]), label="En"))
    NodeConstraints.append(Constraint((x[96] + x[104] + x[128] - 2 * x[129]), label="En"))
    NodeConstraints.append(Constraint((x[98] + x[106] + x[130] - 2 * x[131]), label="En"))
    NodeConstraints.append(Constraint((x[100] + x[108] + x[132] - 2 * x[133]), label="En"))
    NodeConstraints.append(Constraint((x[102] + x[110] + x[134] - 2 * x[135]), label="En"))
    NodeConstraints.append(Constraint((x[68] + x[76] + x[136] - 2 * x[137]), label="En"))
    NodeConstraints.append(Constraint((x[70] + x[78] + x[138] - 2 * x[139]), label="En"))
    NodeConstraints.append(Constraint((x[38] + x[46] + x[140] - 2 * x[141]), label="En"))
    NodeConstraints.append(Constraint((x[7] + x[15] + x[142] - 2 * x[143]), label="En"))
    NodeConstraints.append(Constraint((x[112] + x[120] + x[144] - 2 * x[145]), label="En"))
    NodeConstraints.append(Constraint((x[114] + x[122] + x[146] - 2 * x[147]), label="En"))
    NodeConstraints.append(Constraint((x[116] + x[124] + x[148] - 2 * x[149]), label="En"))
    NodeConstraints.append(Constraint((x[118] + x[126] + x[150] - 2 * x[151]), label="En"))
    NodeConstraints.append(Constraint((x[84] + x[92] + x[152] - 2 * x[153]), label="En"))
    NodeConstraints.append(Constraint((x[86] + x[94] + x[154] - 2 * x[155]), label="En"))
    NodeConstraints.append(Constraint((x[54] + x[62] + x[156] - 2 * x[157]), label="En"))
    NodeConstraints.append(Constraint((x[23] + x[31] + x[158] - 2 * x[159]), label="En"))
    NodeConstraints.append(Constraint((x[128] + x[144] + x[160] - 2 * x[161]), label="En"))
    NodeConstraints.append(Constraint((x[130] + x[146] + x[162] - 2 * x[163]), label="En"))
    NodeConstraints.append(Constraint((x[132] + x[148] + x[164] - 2 * x[165]), label="En"))
    NodeConstraints.append(Constraint((x[134] + x[150] + x[166] - 2 * x[167]), label="En"))
    NodeConstraints.append(Constraint((x[136] + x[152] + x[168] - 2 * x[169]), label="En"))
    NodeConstraints.append(Constraint((x[138] + x[154] + x[170] - 2 * x[171]), label="En"))
    NodeConstraints.append(Constraint((x[140] + x[156] + x[172] - 2 * x[173]), label="En"))
    NodeConstraints.append(Constraint((x[142] + x[158] + x[174] - 2 * x[175]), label="En"))
    NodeConstraints.append(Constraint((x[104] + x[120] + x[176] - 2 * x[177]), label="En"))
    NodeConstraints.append(Constraint((x[106] + x[122] + x[178] - 2 * x[179]), label="En"))
    NodeConstraints.append(Constraint((x[108] + x[124] + x[180] - 2 * x[181]), label="En"))
    NodeConstraints.append(Constraint((x[110] + x[126] + x[182] - 2 * x[183]), label="En"))
    NodeConstraints.append(Constraint((x[76] + x[92] + x[184] - 2 * x[185]), label="En"))
    NodeConstraints.append(Constraint((x[78] + x[94] + x[186] - 2 * x[187]), label="En"))
    NodeConstraints.append(Constraint((x[46] + x[62] + x[188] - 2 * x[189]), label="En"))
    NodeConstraints.append(Constraint((x[15] + x[31] + x[190] - 2 * x[191]), label="En"))
    NodeConstraints = [c * c for c in NodeConstraints]
    constraint_func = We * sum(NodeConstraints)
    cost_func = y[0] * (1 - x[160]) + y[1] * x[160] + y[2] * (1 - x[162]) + y[3] * x[162] + y[4] * (1 - x[164]) + y[5] * \
                x[164] \
                + y[6] * (1 - x[166]) + y[7] * x[166] + y[8] * (1 - x[168]) + y[9] * x[168] + y[10] * (1 - x[170]) + y[
                    11] * x[170] \
                + y[12] * (1 - x[172]) + y[13] * x[172] + y[14] * (1 - x[174]) + y[15] * x[174] + y[16] * (1 - x[176]) + \
                y[17] * x[176] \
                + y[18] * (1 - x[178]) + y[19] * x[178] + y[20] * (1 - x[180]) + y[21] * x[180] + y[22] * (1 - x[182]) + \
                y[23] * x[182] \
                + y[24] * (1 - x[184]) + y[25] * x[184] + y[26] * (1 - x[186]) + y[27] * x[186] + y[28] * (1 - x[188]) + \
                y[29] * x[188] \
                + y[30] * (1 - x[190]) + y[31] * x[190] + y[32] * (1 - x[144]) + y[33] * x[144] + y[34] * (1 - x[146]) + \
                y[35] * x[146] \
                + y[36] * (1 - x[148]) + y[37] * x[148] + y[38] * (1 - x[150]) + y[39] * x[150] + y[40] * (1 - x[152]) + \
                y[41] * x[152] \
                + y[42] * (1 - x[154]) + y[43] * x[154] + y[44] * (1 - x[156]) + y[45] * x[156] + y[46] * (1 - x[158]) + \
                y[47] * x[158] \
                + y[48] * (1 - x[120]) + y[49] * x[120] + y[50] * (1 - x[122]) + y[51] * x[122] + y[52] * (1 - x[124]) + \
                y[53] * x[124] \
                + y[54] * (1 - x[126]) + y[55] * x[126] + y[56] * (1 - x[92]) + y[57] * x[92] + y[58] * (1 - x[94]) + y[
                    59] * x[94] \
                + y[60] * (1 - x[62]) + y[61] * x[62] + y[62] * (1 - x[31]) + y[63] * x[31] + 100 * (
                            x[0] + x[1] + x[2] + x[3] + x[4] + x[5] + x[6] + x[8] + x[9] + x[10] + x[12] + x[16] + x[
                        17] + x[18] + x[20] + x[24])
    H = cost_func + constraint_func
    model = H.compile()
    bqm = model.to_bqm()
    sa = neal.SimulatedAnnealingSampler()
    sampleset = sa.sample(bqm, num_reads=300)
    decoded_samples = model.decode_sampleset(sampleset)
    best_sample = min(decoded_samples, key=lambda x: x.energy)
    result = [0, 0]
    for i in range(K):
        if line[i] != str(best_sample.sample[a[index[i]]]):
            result[0] = result[0] + 1
    raw_solution = best_sample.sample
    decoded_sample = model.decode_sample(raw_solution, vartype='BINARY')
    for c in decoded_sample.constraints(only_broken=True):
        if c[0] == 'E':
            result[1] = 1
    q.put(result)


def thr5(y, line, N, K, n_var, a, We, q, index):
    x = [Binary(a[i]) for i in range(n_var)]
    NodeConstraints = []
    NodeConstraints.append(Constraint((x[0] + x[1] + x[32] - 2 * x[33]), label="En"))
    NodeConstraints.append(Constraint((x[2] + x[3] + x[34] - 2 * x[35]), label="En"))
    NodeConstraints.append(Constraint((x[4] + x[5] + x[36] - 2 * x[37]), label="En"))
    NodeConstraints.append(Constraint((x[6] + x[7] + x[38] - 2 * x[39]), label="En"))
    NodeConstraints.append(Constraint((x[8] + x[9] + x[40] - 2 * x[41]), label="En"))
    NodeConstraints.append(Constraint((x[10] + x[11] + x[42] - 2 * x[43]), label="En"))
    NodeConstraints.append(Constraint((x[12] + x[13] + x[44] - 2 * x[45]), label="En"))
    NodeConstraints.append(Constraint((x[14] + x[15] + x[46] - 2 * x[47]), label="En"))
    NodeConstraints.append(Constraint((x[16] + x[17] + x[48] - 2 * x[49]), label="En"))
    NodeConstraints.append(Constraint((x[18] + x[19] + x[50] - 2 * x[51]), label="En"))
    NodeConstraints.append(Constraint((x[20] + x[21] + x[52] - 2 * x[53]), label="En"))
    NodeConstraints.append(Constraint((x[22] + x[23] + x[54] - 2 * x[55]), label="En"))
    NodeConstraints.append(Constraint((x[24] + x[25] + x[56] - 2 * x[57]), label="En"))
    NodeConstraints.append(Constraint((x[26] + x[27] + x[58] - 2 * x[59]), label="En"))
    NodeConstraints.append(Constraint((x[28] + x[29] + x[60] - 2 * x[61]), label="En"))
    NodeConstraints.append(Constraint((x[30] + x[31] + x[62] - 2 * x[63]), label="En"))
    NodeConstraints.append(Constraint((x[32] + x[34] + x[64] - 2 * x[65]), label="En"))
    NodeConstraints.append(Constraint((x[1] + x[3] + x[66] - 2 * x[67]), label="En"))
    NodeConstraints.append(Constraint((x[36] + x[38] + x[68] - 2 * x[69]), label="En"))
    NodeConstraints.append(Constraint((x[5] + x[7] + x[70] - 2 * x[71]), label="En"))
    NodeConstraints.append(Constraint((x[40] + x[42] + x[72] - 2 * x[73]), label="En"))
    NodeConstraints.append(Constraint((x[9] + x[11] + x[74] - 2 * x[75]), label="En"))
    NodeConstraints.append(Constraint((x[44] + x[46] + x[76] - 2 * x[77]), label="En"))
    NodeConstraints.append(Constraint((x[13] + x[15] + x[78] - 2 * x[79]), label="En"))
    NodeConstraints.append(Constraint((x[48] + x[50] + x[80] - 2 * x[81]), label="En"))
    NodeConstraints.append(Constraint((x[17] + x[19] + x[82] - 2 * x[83]), label="En"))
    NodeConstraints.append(Constraint((x[52] + x[54] + x[84] - 2 * x[85]), label="En"))
    NodeConstraints.append(Constraint((x[21] + x[23] + x[86] - 2 * x[87]), label="En"))
    NodeConstraints.append(Constraint((x[56] + x[58] + x[88] - 2 * x[89]), label="En"))
    NodeConstraints.append(Constraint((x[25] + x[27] + x[90] - 2 * x[91]), label="En"))
    NodeConstraints.append(Constraint((x[60] + x[62] + x[92] - 2 * x[93]), label="En"))
    NodeConstraints.append(Constraint((x[29] + x[31] + x[94] - 2 * x[95]), label="En"))
    NodeConstraints.append(Constraint((x[64] + x[68] + x[96] - 2 * x[97]), label="En"))
    NodeConstraints.append(Constraint((x[66] + x[70] + x[98] - 2 * x[99]), label="En"))
    NodeConstraints.append(Constraint((x[34] + x[38] + x[100] - 2 * x[101]), label="En"))
    NodeConstraints.append(Constraint((x[3] + x[7] + x[102] - 2 * x[103]), label="En"))
    NodeConstraints.append(Constraint((x[72] + x[76] + x[104] - 2 * x[105]), label="En"))
    NodeConstraints.append(Constraint((x[74] + x[78] + x[106] - 2 * x[107]), label="En"))
    NodeConstraints.append(Constraint((x[42] + x[46] + x[108] - 2 * x[109]), label="En"))
    NodeConstraints.append(Constraint((x[11] + x[15] + x[110] - 2 * x[111]), label="En"))
    NodeConstraints.append(Constraint((x[80] + x[84] + x[112] - 2 * x[113]), label="En"))
    NodeConstraints.append(Constraint((x[82] + x[86] + x[114] - 2 * x[115]), label="En"))
    NodeConstraints.append(Constraint((x[50] + x[54] + x[116] - 2 * x[117]), label="En"))
    NodeConstraints.append(Constraint((x[19] + x[23] + x[118] - 2 * x[119]), label="En"))
    NodeConstraints.append(Constraint((x[88] + x[92] + x[120] - 2 * x[121]), label="En"))
    NodeConstraints.append(Constraint((x[90] + x[94] + x[122] - 2 * x[123]), label="En"))
    NodeConstraints.append(Constraint((x[58] + x[62] + x[124] - 2 * x[125]), label="En"))
    NodeConstraints.append(Constraint((x[27] + x[31] + x[126] - 2 * x[127]), label="En"))
    NodeConstraints.append(Constraint((x[96] + x[104] + x[128] - 2 * x[129]), label="En"))
    NodeConstraints.append(Constraint((x[98] + x[106] + x[130] - 2 * x[131]), label="En"))
    NodeConstraints.append(Constraint((x[100] + x[108] + x[132] - 2 * x[133]), label="En"))
    NodeConstraints.append(Constraint((x[102] + x[110] + x[134] - 2 * x[135]), label="En"))
    NodeConstraints.append(Constraint((x[68] + x[76] + x[136] - 2 * x[137]), label="En"))
    NodeConstraints.append(Constraint((x[70] + x[78] + x[138] - 2 * x[139]), label="En"))
    NodeConstraints.append(Constraint((x[38] + x[46] + x[140] - 2 * x[141]), label="En"))
    NodeConstraints.append(Constraint((x[7] + x[15] + x[142] - 2 * x[143]), label="En"))
    NodeConstraints.append(Constraint((x[112] + x[120] + x[144] - 2 * x[145]), label="En"))
    NodeConstraints.append(Constraint((x[114] + x[122] + x[146] - 2 * x[147]), label="En"))
    NodeConstraints.append(Constraint((x[116] + x[124] + x[148] - 2 * x[149]), label="En"))
    NodeConstraints.append(Constraint((x[118] + x[126] + x[150] - 2 * x[151]), label="En"))
    NodeConstraints.append(Constraint((x[84] + x[92] + x[152] - 2 * x[153]), label="En"))
    NodeConstraints.append(Constraint((x[86] + x[94] + x[154] - 2 * x[155]), label="En"))
    NodeConstraints.append(Constraint((x[54] + x[62] + x[156] - 2 * x[157]), label="En"))
    NodeConstraints.append(Constraint((x[23] + x[31] + x[158] - 2 * x[159]), label="En"))
    NodeConstraints.append(Constraint((x[128] + x[144] + x[160] - 2 * x[161]), label="En"))
    NodeConstraints.append(Constraint((x[130] + x[146] + x[162] - 2 * x[163]), label="En"))
    NodeConstraints.append(Constraint((x[132] + x[148] + x[164] - 2 * x[165]), label="En"))
    NodeConstraints.append(Constraint((x[134] + x[150] + x[166] - 2 * x[167]), label="En"))
    NodeConstraints.append(Constraint((x[136] + x[152] + x[168] - 2 * x[169]), label="En"))
    NodeConstraints.append(Constraint((x[138] + x[154] + x[170] - 2 * x[171]), label="En"))
    NodeConstraints.append(Constraint((x[140] + x[156] + x[172] - 2 * x[173]), label="En"))
    NodeConstraints.append(Constraint((x[142] + x[158] + x[174] - 2 * x[175]), label="En"))
    NodeConstraints.append(Constraint((x[104] + x[120] + x[176] - 2 * x[177]), label="En"))
    NodeConstraints.append(Constraint((x[106] + x[122] + x[178] - 2 * x[179]), label="En"))
    NodeConstraints.append(Constraint((x[108] + x[124] + x[180] - 2 * x[181]), label="En"))
    NodeConstraints.append(Constraint((x[110] + x[126] + x[182] - 2 * x[183]), label="En"))
    NodeConstraints.append(Constraint((x[76] + x[92] + x[184] - 2 * x[185]), label="En"))
    NodeConstraints.append(Constraint((x[78] + x[94] + x[186] - 2 * x[187]), label="En"))
    NodeConstraints.append(Constraint((x[46] + x[62] + x[188] - 2 * x[189]), label="En"))
    NodeConstraints.append(Constraint((x[15] + x[31] + x[190] - 2 * x[191]), label="En"))
    NodeConstraints = [c * c for c in NodeConstraints]
    constraint_func = We * sum(NodeConstraints)
    cost_func = y[0] * (1 - x[160]) + y[1] * x[160] + y[2] * (1 - x[162]) + y[3] * x[162] + y[4] * (1 - x[164]) + y[5] * \
                x[164] \
                + y[6] * (1 - x[166]) + y[7] * x[166] + y[8] * (1 - x[168]) + y[9] * x[168] + y[10] * (1 - x[170]) + y[
                    11] * x[170] \
                + y[12] * (1 - x[172]) + y[13] * x[172] + y[14] * (1 - x[174]) + y[15] * x[174] + y[16] * (1 - x[176]) + \
                y[17] * x[176] \
                + y[18] * (1 - x[178]) + y[19] * x[178] + y[20] * (1 - x[180]) + y[21] * x[180] + y[22] * (1 - x[182]) + \
                y[23] * x[182] \
                + y[24] * (1 - x[184]) + y[25] * x[184] + y[26] * (1 - x[186]) + y[27] * x[186] + y[28] * (1 - x[188]) + \
                y[29] * x[188] \
                + y[30] * (1 - x[190]) + y[31] * x[190] + y[32] * (1 - x[144]) + y[33] * x[144] + y[34] * (1 - x[146]) + \
                y[35] * x[146] \
                + y[36] * (1 - x[148]) + y[37] * x[148] + y[38] * (1 - x[150]) + y[39] * x[150] + y[40] * (1 - x[152]) + \
                y[41] * x[152] \
                + y[42] * (1 - x[154]) + y[43] * x[154] + y[44] * (1 - x[156]) + y[45] * x[156] + y[46] * (1 - x[158]) + \
                y[47] * x[158] \
                + y[48] * (1 - x[120]) + y[49] * x[120] + y[50] * (1 - x[122]) + y[51] * x[122] + y[52] * (1 - x[124]) + \
                y[53] * x[124] \
                + y[54] * (1 - x[126]) + y[55] * x[126] + y[56] * (1 - x[92]) + y[57] * x[92] + y[58] * (1 - x[94]) + y[
                    59] * x[94] \
                + y[60] * (1 - x[62]) + y[61] * x[62] + y[62] * (1 - x[31]) + y[63] * x[31] + 100 * (
                            x[0] + x[1] + x[2] + x[3] + x[4] + x[5] + x[6] + x[8] + x[9] + x[10] + x[12] + x[16] + x[
                        17] + x[18] + x[20] + x[24])
    H = cost_func + constraint_func
    model = H.compile()
    bqm = model.to_bqm()
    sa = neal.SimulatedAnnealingSampler()
    sampleset = sa.sample(bqm, num_reads=300)
    decoded_samples = model.decode_sampleset(sampleset)
    best_sample = min(decoded_samples, key=lambda x: x.energy)
    result = [0, 0]
    for i in range(K):
        if line[i] != str(best_sample.sample[a[index[i]]]):
            result[0] = result[0] + 1
    raw_solution = best_sample.sample
    decoded_sample = model.decode_sample(raw_solution, vartype='BINARY')
    for c in decoded_sample.constraints(only_broken=True):
        if c[0] == 'E':
            result[1] = 1
    q.put(result)


def thr6(y, line, N, K, n_var, a, We, q, index):
    x = [Binary(a[i]) for i in range(n_var)]
    NodeConstraints = []
    NodeConstraints.append(Constraint((x[0] + x[1] + x[32] - 2 * x[33]), label="En"))
    NodeConstraints.append(Constraint((x[2] + x[3] + x[34] - 2 * x[35]), label="En"))
    NodeConstraints.append(Constraint((x[4] + x[5] + x[36] - 2 * x[37]), label="En"))
    NodeConstraints.append(Constraint((x[6] + x[7] + x[38] - 2 * x[39]), label="En"))
    NodeConstraints.append(Constraint((x[8] + x[9] + x[40] - 2 * x[41]), label="En"))
    NodeConstraints.append(Constraint((x[10] + x[11] + x[42] - 2 * x[43]), label="En"))
    NodeConstraints.append(Constraint((x[12] + x[13] + x[44] - 2 * x[45]), label="En"))
    NodeConstraints.append(Constraint((x[14] + x[15] + x[46] - 2 * x[47]), label="En"))
    NodeConstraints.append(Constraint((x[16] + x[17] + x[48] - 2 * x[49]), label="En"))
    NodeConstraints.append(Constraint((x[18] + x[19] + x[50] - 2 * x[51]), label="En"))
    NodeConstraints.append(Constraint((x[20] + x[21] + x[52] - 2 * x[53]), label="En"))
    NodeConstraints.append(Constraint((x[22] + x[23] + x[54] - 2 * x[55]), label="En"))
    NodeConstraints.append(Constraint((x[24] + x[25] + x[56] - 2 * x[57]), label="En"))
    NodeConstraints.append(Constraint((x[26] + x[27] + x[58] - 2 * x[59]), label="En"))
    NodeConstraints.append(Constraint((x[28] + x[29] + x[60] - 2 * x[61]), label="En"))
    NodeConstraints.append(Constraint((x[30] + x[31] + x[62] - 2 * x[63]), label="En"))
    NodeConstraints.append(Constraint((x[32] + x[34] + x[64] - 2 * x[65]), label="En"))
    NodeConstraints.append(Constraint((x[1] + x[3] + x[66] - 2 * x[67]), label="En"))
    NodeConstraints.append(Constraint((x[36] + x[38] + x[68] - 2 * x[69]), label="En"))
    NodeConstraints.append(Constraint((x[5] + x[7] + x[70] - 2 * x[71]), label="En"))
    NodeConstraints.append(Constraint((x[40] + x[42] + x[72] - 2 * x[73]), label="En"))
    NodeConstraints.append(Constraint((x[9] + x[11] + x[74] - 2 * x[75]), label="En"))
    NodeConstraints.append(Constraint((x[44] + x[46] + x[76] - 2 * x[77]), label="En"))
    NodeConstraints.append(Constraint((x[13] + x[15] + x[78] - 2 * x[79]), label="En"))
    NodeConstraints.append(Constraint((x[48] + x[50] + x[80] - 2 * x[81]), label="En"))
    NodeConstraints.append(Constraint((x[17] + x[19] + x[82] - 2 * x[83]), label="En"))
    NodeConstraints.append(Constraint((x[52] + x[54] + x[84] - 2 * x[85]), label="En"))
    NodeConstraints.append(Constraint((x[21] + x[23] + x[86] - 2 * x[87]), label="En"))
    NodeConstraints.append(Constraint((x[56] + x[58] + x[88] - 2 * x[89]), label="En"))
    NodeConstraints.append(Constraint((x[25] + x[27] + x[90] - 2 * x[91]), label="En"))
    NodeConstraints.append(Constraint((x[60] + x[62] + x[92] - 2 * x[93]), label="En"))
    NodeConstraints.append(Constraint((x[29] + x[31] + x[94] - 2 * x[95]), label="En"))
    NodeConstraints.append(Constraint((x[64] + x[68] + x[96] - 2 * x[97]), label="En"))
    NodeConstraints.append(Constraint((x[66] + x[70] + x[98] - 2 * x[99]), label="En"))
    NodeConstraints.append(Constraint((x[34] + x[38] + x[100] - 2 * x[101]), label="En"))
    NodeConstraints.append(Constraint((x[3] + x[7] + x[102] - 2 * x[103]), label="En"))
    NodeConstraints.append(Constraint((x[72] + x[76] + x[104] - 2 * x[105]), label="En"))
    NodeConstraints.append(Constraint((x[74] + x[78] + x[106] - 2 * x[107]), label="En"))
    NodeConstraints.append(Constraint((x[42] + x[46] + x[108] - 2 * x[109]), label="En"))
    NodeConstraints.append(Constraint((x[11] + x[15] + x[110] - 2 * x[111]), label="En"))
    NodeConstraints.append(Constraint((x[80] + x[84] + x[112] - 2 * x[113]), label="En"))
    NodeConstraints.append(Constraint((x[82] + x[86] + x[114] - 2 * x[115]), label="En"))
    NodeConstraints.append(Constraint((x[50] + x[54] + x[116] - 2 * x[117]), label="En"))
    NodeConstraints.append(Constraint((x[19] + x[23] + x[118] - 2 * x[119]), label="En"))
    NodeConstraints.append(Constraint((x[88] + x[92] + x[120] - 2 * x[121]), label="En"))
    NodeConstraints.append(Constraint((x[90] + x[94] + x[122] - 2 * x[123]), label="En"))
    NodeConstraints.append(Constraint((x[58] + x[62] + x[124] - 2 * x[125]), label="En"))
    NodeConstraints.append(Constraint((x[27] + x[31] + x[126] - 2 * x[127]), label="En"))
    NodeConstraints.append(Constraint((x[96] + x[104] + x[128] - 2 * x[129]), label="En"))
    NodeConstraints.append(Constraint((x[98] + x[106] + x[130] - 2 * x[131]), label="En"))
    NodeConstraints.append(Constraint((x[100] + x[108] + x[132] - 2 * x[133]), label="En"))
    NodeConstraints.append(Constraint((x[102] + x[110] + x[134] - 2 * x[135]), label="En"))
    NodeConstraints.append(Constraint((x[68] + x[76] + x[136] - 2 * x[137]), label="En"))
    NodeConstraints.append(Constraint((x[70] + x[78] + x[138] - 2 * x[139]), label="En"))
    NodeConstraints.append(Constraint((x[38] + x[46] + x[140] - 2 * x[141]), label="En"))
    NodeConstraints.append(Constraint((x[7] + x[15] + x[142] - 2 * x[143]), label="En"))
    NodeConstraints.append(Constraint((x[112] + x[120] + x[144] - 2 * x[145]), label="En"))
    NodeConstraints.append(Constraint((x[114] + x[122] + x[146] - 2 * x[147]), label="En"))
    NodeConstraints.append(Constraint((x[116] + x[124] + x[148] - 2 * x[149]), label="En"))
    NodeConstraints.append(Constraint((x[118] + x[126] + x[150] - 2 * x[151]), label="En"))
    NodeConstraints.append(Constraint((x[84] + x[92] + x[152] - 2 * x[153]), label="En"))
    NodeConstraints.append(Constraint((x[86] + x[94] + x[154] - 2 * x[155]), label="En"))
    NodeConstraints.append(Constraint((x[54] + x[62] + x[156] - 2 * x[157]), label="En"))
    NodeConstraints.append(Constraint((x[23] + x[31] + x[158] - 2 * x[159]), label="En"))
    NodeConstraints.append(Constraint((x[128] + x[144] + x[160] - 2 * x[161]), label="En"))
    NodeConstraints.append(Constraint((x[130] + x[146] + x[162] - 2 * x[163]), label="En"))
    NodeConstraints.append(Constraint((x[132] + x[148] + x[164] - 2 * x[165]), label="En"))
    NodeConstraints.append(Constraint((x[134] + x[150] + x[166] - 2 * x[167]), label="En"))
    NodeConstraints.append(Constraint((x[136] + x[152] + x[168] - 2 * x[169]), label="En"))
    NodeConstraints.append(Constraint((x[138] + x[154] + x[170] - 2 * x[171]), label="En"))
    NodeConstraints.append(Constraint((x[140] + x[156] + x[172] - 2 * x[173]), label="En"))
    NodeConstraints.append(Constraint((x[142] + x[158] + x[174] - 2 * x[175]), label="En"))
    NodeConstraints.append(Constraint((x[104] + x[120] + x[176] - 2 * x[177]), label="En"))
    NodeConstraints.append(Constraint((x[106] + x[122] + x[178] - 2 * x[179]), label="En"))
    NodeConstraints.append(Constraint((x[108] + x[124] + x[180] - 2 * x[181]), label="En"))
    NodeConstraints.append(Constraint((x[110] + x[126] + x[182] - 2 * x[183]), label="En"))
    NodeConstraints.append(Constraint((x[76] + x[92] + x[184] - 2 * x[185]), label="En"))
    NodeConstraints.append(Constraint((x[78] + x[94] + x[186] - 2 * x[187]), label="En"))
    NodeConstraints.append(Constraint((x[46] + x[62] + x[188] - 2 * x[189]), label="En"))
    NodeConstraints.append(Constraint((x[15] + x[31] + x[190] - 2 * x[191]), label="En"))
    NodeConstraints = [c * c for c in NodeConstraints]
    constraint_func = We * sum(NodeConstraints)
    cost_func = y[0] * (1 - x[160]) + y[1] * x[160] + y[2] * (1 - x[162]) + y[3] * x[162] + y[4] * (1 - x[164]) + y[5] * \
                x[164] \
                + y[6] * (1 - x[166]) + y[7] * x[166] + y[8] * (1 - x[168]) + y[9] * x[168] + y[10] * (1 - x[170]) + y[
                    11] * x[170] \
                + y[12] * (1 - x[172]) + y[13] * x[172] + y[14] * (1 - x[174]) + y[15] * x[174] + y[16] * (1 - x[176]) + \
                y[17] * x[176] \
                + y[18] * (1 - x[178]) + y[19] * x[178] + y[20] * (1 - x[180]) + y[21] * x[180] + y[22] * (1 - x[182]) + \
                y[23] * x[182] \
                + y[24] * (1 - x[184]) + y[25] * x[184] + y[26] * (1 - x[186]) + y[27] * x[186] + y[28] * (1 - x[188]) + \
                y[29] * x[188] \
                + y[30] * (1 - x[190]) + y[31] * x[190] + y[32] * (1 - x[144]) + y[33] * x[144] + y[34] * (1 - x[146]) + \
                y[35] * x[146] \
                + y[36] * (1 - x[148]) + y[37] * x[148] + y[38] * (1 - x[150]) + y[39] * x[150] + y[40] * (1 - x[152]) + \
                y[41] * x[152] \
                + y[42] * (1 - x[154]) + y[43] * x[154] + y[44] * (1 - x[156]) + y[45] * x[156] + y[46] * (1 - x[158]) + \
                y[47] * x[158] \
                + y[48] * (1 - x[120]) + y[49] * x[120] + y[50] * (1 - x[122]) + y[51] * x[122] + y[52] * (1 - x[124]) + \
                y[53] * x[124] \
                + y[54] * (1 - x[126]) + y[55] * x[126] + y[56] * (1 - x[92]) + y[57] * x[92] + y[58] * (1 - x[94]) + y[
                    59] * x[94] \
                + y[60] * (1 - x[62]) + y[61] * x[62] + y[62] * (1 - x[31]) + y[63] * x[31] + 100 * (
                            x[0] + x[1] + x[2] + x[3] + x[4] + x[5] + x[6] + x[8] + x[9] + x[10] + x[12] + x[16] + x[
                        17] + x[18] + x[20] + x[24])
    H = cost_func + constraint_func
    model = H.compile()
    bqm = model.to_bqm()
    sa = neal.SimulatedAnnealingSampler()
    sampleset = sa.sample(bqm, num_reads=300)
    decoded_samples = model.decode_sampleset(sampleset)
    best_sample = min(decoded_samples, key=lambda x: x.energy)
    result = [0, 0]
    for i in range(K):
        if line[i] != str(best_sample.sample[a[index[i]]]):
            result[0] = result[0] + 1
    raw_solution = best_sample.sample
    decoded_sample = model.decode_sample(raw_solution, vartype='BINARY')
    for c in decoded_sample.constraints(only_broken=True):
        if c[0] == 'E':
            result[1] = 1
    q.put(result)


def thr7(y, line, N, K, n_var, a, We, q, index):
    x = [Binary(a[i]) for i in range(n_var)]
    NodeConstraints = []
    NodeConstraints.append(Constraint((x[0] + x[1] + x[32] - 2 * x[33]), label="En"))
    NodeConstraints.append(Constraint((x[2] + x[3] + x[34] - 2 * x[35]), label="En"))
    NodeConstraints.append(Constraint((x[4] + x[5] + x[36] - 2 * x[37]), label="En"))
    NodeConstraints.append(Constraint((x[6] + x[7] + x[38] - 2 * x[39]), label="En"))
    NodeConstraints.append(Constraint((x[8] + x[9] + x[40] - 2 * x[41]), label="En"))
    NodeConstraints.append(Constraint((x[10] + x[11] + x[42] - 2 * x[43]), label="En"))
    NodeConstraints.append(Constraint((x[12] + x[13] + x[44] - 2 * x[45]), label="En"))
    NodeConstraints.append(Constraint((x[14] + x[15] + x[46] - 2 * x[47]), label="En"))
    NodeConstraints.append(Constraint((x[16] + x[17] + x[48] - 2 * x[49]), label="En"))
    NodeConstraints.append(Constraint((x[18] + x[19] + x[50] - 2 * x[51]), label="En"))
    NodeConstraints.append(Constraint((x[20] + x[21] + x[52] - 2 * x[53]), label="En"))
    NodeConstraints.append(Constraint((x[22] + x[23] + x[54] - 2 * x[55]), label="En"))
    NodeConstraints.append(Constraint((x[24] + x[25] + x[56] - 2 * x[57]), label="En"))
    NodeConstraints.append(Constraint((x[26] + x[27] + x[58] - 2 * x[59]), label="En"))
    NodeConstraints.append(Constraint((x[28] + x[29] + x[60] - 2 * x[61]), label="En"))
    NodeConstraints.append(Constraint((x[30] + x[31] + x[62] - 2 * x[63]), label="En"))
    NodeConstraints.append(Constraint((x[32] + x[34] + x[64] - 2 * x[65]), label="En"))
    NodeConstraints.append(Constraint((x[1] + x[3] + x[66] - 2 * x[67]), label="En"))
    NodeConstraints.append(Constraint((x[36] + x[38] + x[68] - 2 * x[69]), label="En"))
    NodeConstraints.append(Constraint((x[5] + x[7] + x[70] - 2 * x[71]), label="En"))
    NodeConstraints.append(Constraint((x[40] + x[42] + x[72] - 2 * x[73]), label="En"))
    NodeConstraints.append(Constraint((x[9] + x[11] + x[74] - 2 * x[75]), label="En"))
    NodeConstraints.append(Constraint((x[44] + x[46] + x[76] - 2 * x[77]), label="En"))
    NodeConstraints.append(Constraint((x[13] + x[15] + x[78] - 2 * x[79]), label="En"))
    NodeConstraints.append(Constraint((x[48] + x[50] + x[80] - 2 * x[81]), label="En"))
    NodeConstraints.append(Constraint((x[17] + x[19] + x[82] - 2 * x[83]), label="En"))
    NodeConstraints.append(Constraint((x[52] + x[54] + x[84] - 2 * x[85]), label="En"))
    NodeConstraints.append(Constraint((x[21] + x[23] + x[86] - 2 * x[87]), label="En"))
    NodeConstraints.append(Constraint((x[56] + x[58] + x[88] - 2 * x[89]), label="En"))
    NodeConstraints.append(Constraint((x[25] + x[27] + x[90] - 2 * x[91]), label="En"))
    NodeConstraints.append(Constraint((x[60] + x[62] + x[92] - 2 * x[93]), label="En"))
    NodeConstraints.append(Constraint((x[29] + x[31] + x[94] - 2 * x[95]), label="En"))
    NodeConstraints.append(Constraint((x[64] + x[68] + x[96] - 2 * x[97]), label="En"))
    NodeConstraints.append(Constraint((x[66] + x[70] + x[98] - 2 * x[99]), label="En"))
    NodeConstraints.append(Constraint((x[34] + x[38] + x[100] - 2 * x[101]), label="En"))
    NodeConstraints.append(Constraint((x[3] + x[7] + x[102] - 2 * x[103]), label="En"))
    NodeConstraints.append(Constraint((x[72] + x[76] + x[104] - 2 * x[105]), label="En"))
    NodeConstraints.append(Constraint((x[74] + x[78] + x[106] - 2 * x[107]), label="En"))
    NodeConstraints.append(Constraint((x[42] + x[46] + x[108] - 2 * x[109]), label="En"))
    NodeConstraints.append(Constraint((x[11] + x[15] + x[110] - 2 * x[111]), label="En"))
    NodeConstraints.append(Constraint((x[80] + x[84] + x[112] - 2 * x[113]), label="En"))
    NodeConstraints.append(Constraint((x[82] + x[86] + x[114] - 2 * x[115]), label="En"))
    NodeConstraints.append(Constraint((x[50] + x[54] + x[116] - 2 * x[117]), label="En"))
    NodeConstraints.append(Constraint((x[19] + x[23] + x[118] - 2 * x[119]), label="En"))
    NodeConstraints.append(Constraint((x[88] + x[92] + x[120] - 2 * x[121]), label="En"))
    NodeConstraints.append(Constraint((x[90] + x[94] + x[122] - 2 * x[123]), label="En"))
    NodeConstraints.append(Constraint((x[58] + x[62] + x[124] - 2 * x[125]), label="En"))
    NodeConstraints.append(Constraint((x[27] + x[31] + x[126] - 2 * x[127]), label="En"))
    NodeConstraints.append(Constraint((x[96] + x[104] + x[128] - 2 * x[129]), label="En"))
    NodeConstraints.append(Constraint((x[98] + x[106] + x[130] - 2 * x[131]), label="En"))
    NodeConstraints.append(Constraint((x[100] + x[108] + x[132] - 2 * x[133]), label="En"))
    NodeConstraints.append(Constraint((x[102] + x[110] + x[134] - 2 * x[135]), label="En"))
    NodeConstraints.append(Constraint((x[68] + x[76] + x[136] - 2 * x[137]), label="En"))
    NodeConstraints.append(Constraint((x[70] + x[78] + x[138] - 2 * x[139]), label="En"))
    NodeConstraints.append(Constraint((x[38] + x[46] + x[140] - 2 * x[141]), label="En"))
    NodeConstraints.append(Constraint((x[7] + x[15] + x[142] - 2 * x[143]), label="En"))
    NodeConstraints.append(Constraint((x[112] + x[120] + x[144] - 2 * x[145]), label="En"))
    NodeConstraints.append(Constraint((x[114] + x[122] + x[146] - 2 * x[147]), label="En"))
    NodeConstraints.append(Constraint((x[116] + x[124] + x[148] - 2 * x[149]), label="En"))
    NodeConstraints.append(Constraint((x[118] + x[126] + x[150] - 2 * x[151]), label="En"))
    NodeConstraints.append(Constraint((x[84] + x[92] + x[152] - 2 * x[153]), label="En"))
    NodeConstraints.append(Constraint((x[86] + x[94] + x[154] - 2 * x[155]), label="En"))
    NodeConstraints.append(Constraint((x[54] + x[62] + x[156] - 2 * x[157]), label="En"))
    NodeConstraints.append(Constraint((x[23] + x[31] + x[158] - 2 * x[159]), label="En"))
    NodeConstraints.append(Constraint((x[128] + x[144] + x[160] - 2 * x[161]), label="En"))
    NodeConstraints.append(Constraint((x[130] + x[146] + x[162] - 2 * x[163]), label="En"))
    NodeConstraints.append(Constraint((x[132] + x[148] + x[164] - 2 * x[165]), label="En"))
    NodeConstraints.append(Constraint((x[134] + x[150] + x[166] - 2 * x[167]), label="En"))
    NodeConstraints.append(Constraint((x[136] + x[152] + x[168] - 2 * x[169]), label="En"))
    NodeConstraints.append(Constraint((x[138] + x[154] + x[170] - 2 * x[171]), label="En"))
    NodeConstraints.append(Constraint((x[140] + x[156] + x[172] - 2 * x[173]), label="En"))
    NodeConstraints.append(Constraint((x[142] + x[158] + x[174] - 2 * x[175]), label="En"))
    NodeConstraints.append(Constraint((x[104] + x[120] + x[176] - 2 * x[177]), label="En"))
    NodeConstraints.append(Constraint((x[106] + x[122] + x[178] - 2 * x[179]), label="En"))
    NodeConstraints.append(Constraint((x[108] + x[124] + x[180] - 2 * x[181]), label="En"))
    NodeConstraints.append(Constraint((x[110] + x[126] + x[182] - 2 * x[183]), label="En"))
    NodeConstraints.append(Constraint((x[76] + x[92] + x[184] - 2 * x[185]), label="En"))
    NodeConstraints.append(Constraint((x[78] + x[94] + x[186] - 2 * x[187]), label="En"))
    NodeConstraints.append(Constraint((x[46] + x[62] + x[188] - 2 * x[189]), label="En"))
    NodeConstraints.append(Constraint((x[15] + x[31] + x[190] - 2 * x[191]), label="En"))
    NodeConstraints = [c * c for c in NodeConstraints]
    constraint_func = We * sum(NodeConstraints)
    cost_func = y[0] * (1 - x[160]) + y[1] * x[160] + y[2] * (1 - x[162]) + y[3] * x[162] + y[4] * (1 - x[164]) + y[5] * \
                x[164] \
                + y[6] * (1 - x[166]) + y[7] * x[166] + y[8] * (1 - x[168]) + y[9] * x[168] + y[10] * (1 - x[170]) + y[
                    11] * x[170] \
                + y[12] * (1 - x[172]) + y[13] * x[172] + y[14] * (1 - x[174]) + y[15] * x[174] + y[16] * (1 - x[176]) + \
                y[17] * x[176] \
                + y[18] * (1 - x[178]) + y[19] * x[178] + y[20] * (1 - x[180]) + y[21] * x[180] + y[22] * (1 - x[182]) + \
                y[23] * x[182] \
                + y[24] * (1 - x[184]) + y[25] * x[184] + y[26] * (1 - x[186]) + y[27] * x[186] + y[28] * (1 - x[188]) + \
                y[29] * x[188] \
                + y[30] * (1 - x[190]) + y[31] * x[190] + y[32] * (1 - x[144]) + y[33] * x[144] + y[34] * (1 - x[146]) + \
                y[35] * x[146] \
                + y[36] * (1 - x[148]) + y[37] * x[148] + y[38] * (1 - x[150]) + y[39] * x[150] + y[40] * (1 - x[152]) + \
                y[41] * x[152] \
                + y[42] * (1 - x[154]) + y[43] * x[154] + y[44] * (1 - x[156]) + y[45] * x[156] + y[46] * (1 - x[158]) + \
                y[47] * x[158] \
                + y[48] * (1 - x[120]) + y[49] * x[120] + y[50] * (1 - x[122]) + y[51] * x[122] + y[52] * (1 - x[124]) + \
                y[53] * x[124] \
                + y[54] * (1 - x[126]) + y[55] * x[126] + y[56] * (1 - x[92]) + y[57] * x[92] + y[58] * (1 - x[94]) + y[
                    59] * x[94] \
                + y[60] * (1 - x[62]) + y[61] * x[62] + y[62] * (1 - x[31]) + y[63] * x[31] + 100 * (
                            x[0] + x[1] + x[2] + x[3] + x[4] + x[5] + x[6] + x[8] + x[9] + x[10] + x[12] + x[16] + x[
                        17] + x[18] + x[20] + x[24])
    H = cost_func + constraint_func
    model = H.compile()
    bqm = model.to_bqm()
    sa = neal.SimulatedAnnealingSampler()
    sampleset = sa.sample(bqm, num_reads=300)
    decoded_samples = model.decode_sampleset(sampleset)
    best_sample = min(decoded_samples, key=lambda x: x.energy)
    result = [0, 0]
    for i in range(K):
        if line[i] != str(best_sample.sample[a[index[i]]]):
            result[0] = result[0] + 1
    raw_solution = best_sample.sample
    decoded_sample = model.decode_sample(raw_solution, vartype='BINARY')
    for c in decoded_sample.constraints(only_broken=True):
        if c[0] == 'E':
            result[1] = 1
    q.put(result)


def thr8(y, line, N, K, n_var, a, We, q, index):
    x = [Binary(a[i]) for i in range(n_var)]
    NodeConstraints = []
    NodeConstraints.append(Constraint((x[0] + x[1] + x[32] - 2 * x[33]), label="En"))
    NodeConstraints.append(Constraint((x[2] + x[3] + x[34] - 2 * x[35]), label="En"))
    NodeConstraints.append(Constraint((x[4] + x[5] + x[36] - 2 * x[37]), label="En"))
    NodeConstraints.append(Constraint((x[6] + x[7] + x[38] - 2 * x[39]), label="En"))
    NodeConstraints.append(Constraint((x[8] + x[9] + x[40] - 2 * x[41]), label="En"))
    NodeConstraints.append(Constraint((x[10] + x[11] + x[42] - 2 * x[43]), label="En"))
    NodeConstraints.append(Constraint((x[12] + x[13] + x[44] - 2 * x[45]), label="En"))
    NodeConstraints.append(Constraint((x[14] + x[15] + x[46] - 2 * x[47]), label="En"))
    NodeConstraints.append(Constraint((x[16] + x[17] + x[48] - 2 * x[49]), label="En"))
    NodeConstraints.append(Constraint((x[18] + x[19] + x[50] - 2 * x[51]), label="En"))
    NodeConstraints.append(Constraint((x[20] + x[21] + x[52] - 2 * x[53]), label="En"))
    NodeConstraints.append(Constraint((x[22] + x[23] + x[54] - 2 * x[55]), label="En"))
    NodeConstraints.append(Constraint((x[24] + x[25] + x[56] - 2 * x[57]), label="En"))
    NodeConstraints.append(Constraint((x[26] + x[27] + x[58] - 2 * x[59]), label="En"))
    NodeConstraints.append(Constraint((x[28] + x[29] + x[60] - 2 * x[61]), label="En"))
    NodeConstraints.append(Constraint((x[30] + x[31] + x[62] - 2 * x[63]), label="En"))
    NodeConstraints.append(Constraint((x[32] + x[34] + x[64] - 2 * x[65]), label="En"))
    NodeConstraints.append(Constraint((x[1] + x[3] + x[66] - 2 * x[67]), label="En"))
    NodeConstraints.append(Constraint((x[36] + x[38] + x[68] - 2 * x[69]), label="En"))
    NodeConstraints.append(Constraint((x[5] + x[7] + x[70] - 2 * x[71]), label="En"))
    NodeConstraints.append(Constraint((x[40] + x[42] + x[72] - 2 * x[73]), label="En"))
    NodeConstraints.append(Constraint((x[9] + x[11] + x[74] - 2 * x[75]), label="En"))
    NodeConstraints.append(Constraint((x[44] + x[46] + x[76] - 2 * x[77]), label="En"))
    NodeConstraints.append(Constraint((x[13] + x[15] + x[78] - 2 * x[79]), label="En"))
    NodeConstraints.append(Constraint((x[48] + x[50] + x[80] - 2 * x[81]), label="En"))
    NodeConstraints.append(Constraint((x[17] + x[19] + x[82] - 2 * x[83]), label="En"))
    NodeConstraints.append(Constraint((x[52] + x[54] + x[84] - 2 * x[85]), label="En"))
    NodeConstraints.append(Constraint((x[21] + x[23] + x[86] - 2 * x[87]), label="En"))
    NodeConstraints.append(Constraint((x[56] + x[58] + x[88] - 2 * x[89]), label="En"))
    NodeConstraints.append(Constraint((x[25] + x[27] + x[90] - 2 * x[91]), label="En"))
    NodeConstraints.append(Constraint((x[60] + x[62] + x[92] - 2 * x[93]), label="En"))
    NodeConstraints.append(Constraint((x[29] + x[31] + x[94] - 2 * x[95]), label="En"))
    NodeConstraints.append(Constraint((x[64] + x[68] + x[96] - 2 * x[97]), label="En"))
    NodeConstraints.append(Constraint((x[66] + x[70] + x[98] - 2 * x[99]), label="En"))
    NodeConstraints.append(Constraint((x[34] + x[38] + x[100] - 2 * x[101]), label="En"))
    NodeConstraints.append(Constraint((x[3] + x[7] + x[102] - 2 * x[103]), label="En"))
    NodeConstraints.append(Constraint((x[72] + x[76] + x[104] - 2 * x[105]), label="En"))
    NodeConstraints.append(Constraint((x[74] + x[78] + x[106] - 2 * x[107]), label="En"))
    NodeConstraints.append(Constraint((x[42] + x[46] + x[108] - 2 * x[109]), label="En"))
    NodeConstraints.append(Constraint((x[11] + x[15] + x[110] - 2 * x[111]), label="En"))
    NodeConstraints.append(Constraint((x[80] + x[84] + x[112] - 2 * x[113]), label="En"))
    NodeConstraints.append(Constraint((x[82] + x[86] + x[114] - 2 * x[115]), label="En"))
    NodeConstraints.append(Constraint((x[50] + x[54] + x[116] - 2 * x[117]), label="En"))
    NodeConstraints.append(Constraint((x[19] + x[23] + x[118] - 2 * x[119]), label="En"))
    NodeConstraints.append(Constraint((x[88] + x[92] + x[120] - 2 * x[121]), label="En"))
    NodeConstraints.append(Constraint((x[90] + x[94] + x[122] - 2 * x[123]), label="En"))
    NodeConstraints.append(Constraint((x[58] + x[62] + x[124] - 2 * x[125]), label="En"))
    NodeConstraints.append(Constraint((x[27] + x[31] + x[126] - 2 * x[127]), label="En"))
    NodeConstraints.append(Constraint((x[96] + x[104] + x[128] - 2 * x[129]), label="En"))
    NodeConstraints.append(Constraint((x[98] + x[106] + x[130] - 2 * x[131]), label="En"))
    NodeConstraints.append(Constraint((x[100] + x[108] + x[132] - 2 * x[133]), label="En"))
    NodeConstraints.append(Constraint((x[102] + x[110] + x[134] - 2 * x[135]), label="En"))
    NodeConstraints.append(Constraint((x[68] + x[76] + x[136] - 2 * x[137]), label="En"))
    NodeConstraints.append(Constraint((x[70] + x[78] + x[138] - 2 * x[139]), label="En"))
    NodeConstraints.append(Constraint((x[38] + x[46] + x[140] - 2 * x[141]), label="En"))
    NodeConstraints.append(Constraint((x[7] + x[15] + x[142] - 2 * x[143]), label="En"))
    NodeConstraints.append(Constraint((x[112] + x[120] + x[144] - 2 * x[145]), label="En"))
    NodeConstraints.append(Constraint((x[114] + x[122] + x[146] - 2 * x[147]), label="En"))
    NodeConstraints.append(Constraint((x[116] + x[124] + x[148] - 2 * x[149]), label="En"))
    NodeConstraints.append(Constraint((x[118] + x[126] + x[150] - 2 * x[151]), label="En"))
    NodeConstraints.append(Constraint((x[84] + x[92] + x[152] - 2 * x[153]), label="En"))
    NodeConstraints.append(Constraint((x[86] + x[94] + x[154] - 2 * x[155]), label="En"))
    NodeConstraints.append(Constraint((x[54] + x[62] + x[156] - 2 * x[157]), label="En"))
    NodeConstraints.append(Constraint((x[23] + x[31] + x[158] - 2 * x[159]), label="En"))
    NodeConstraints.append(Constraint((x[128] + x[144] + x[160] - 2 * x[161]), label="En"))
    NodeConstraints.append(Constraint((x[130] + x[146] + x[162] - 2 * x[163]), label="En"))
    NodeConstraints.append(Constraint((x[132] + x[148] + x[164] - 2 * x[165]), label="En"))
    NodeConstraints.append(Constraint((x[134] + x[150] + x[166] - 2 * x[167]), label="En"))
    NodeConstraints.append(Constraint((x[136] + x[152] + x[168] - 2 * x[169]), label="En"))
    NodeConstraints.append(Constraint((x[138] + x[154] + x[170] - 2 * x[171]), label="En"))
    NodeConstraints.append(Constraint((x[140] + x[156] + x[172] - 2 * x[173]), label="En"))
    NodeConstraints.append(Constraint((x[142] + x[158] + x[174] - 2 * x[175]), label="En"))
    NodeConstraints.append(Constraint((x[104] + x[120] + x[176] - 2 * x[177]), label="En"))
    NodeConstraints.append(Constraint((x[106] + x[122] + x[178] - 2 * x[179]), label="En"))
    NodeConstraints.append(Constraint((x[108] + x[124] + x[180] - 2 * x[181]), label="En"))
    NodeConstraints.append(Constraint((x[110] + x[126] + x[182] - 2 * x[183]), label="En"))
    NodeConstraints.append(Constraint((x[76] + x[92] + x[184] - 2 * x[185]), label="En"))
    NodeConstraints.append(Constraint((x[78] + x[94] + x[186] - 2 * x[187]), label="En"))
    NodeConstraints.append(Constraint((x[46] + x[62] + x[188] - 2 * x[189]), label="En"))
    NodeConstraints.append(Constraint((x[15] + x[31] + x[190] - 2 * x[191]), label="En"))
    NodeConstraints = [c * c for c in NodeConstraints]
    constraint_func = We * sum(NodeConstraints)
    cost_func = y[0] * (1 - x[160]) + y[1] * x[160] + y[2] * (1 - x[162]) + y[3] * x[162] + y[4] * (1 - x[164]) + y[5] * \
                x[164] \
                + y[6] * (1 - x[166]) + y[7] * x[166] + y[8] * (1 - x[168]) + y[9] * x[168] + y[10] * (1 - x[170]) + y[
                    11] * x[170] \
                + y[12] * (1 - x[172]) + y[13] * x[172] + y[14] * (1 - x[174]) + y[15] * x[174] + y[16] * (1 - x[176]) + \
                y[17] * x[176] \
                + y[18] * (1 - x[178]) + y[19] * x[178] + y[20] * (1 - x[180]) + y[21] * x[180] + y[22] * (1 - x[182]) + \
                y[23] * x[182] \
                + y[24] * (1 - x[184]) + y[25] * x[184] + y[26] * (1 - x[186]) + y[27] * x[186] + y[28] * (1 - x[188]) + \
                y[29] * x[188] \
                + y[30] * (1 - x[190]) + y[31] * x[190] + y[32] * (1 - x[144]) + y[33] * x[144] + y[34] * (1 - x[146]) + \
                y[35] * x[146] \
                + y[36] * (1 - x[148]) + y[37] * x[148] + y[38] * (1 - x[150]) + y[39] * x[150] + y[40] * (1 - x[152]) + \
                y[41] * x[152] \
                + y[42] * (1 - x[154]) + y[43] * x[154] + y[44] * (1 - x[156]) + y[45] * x[156] + y[46] * (1 - x[158]) + \
                y[47] * x[158] \
                + y[48] * (1 - x[120]) + y[49] * x[120] + y[50] * (1 - x[122]) + y[51] * x[122] + y[52] * (1 - x[124]) + \
                y[53] * x[124] \
                + y[54] * (1 - x[126]) + y[55] * x[126] + y[56] * (1 - x[92]) + y[57] * x[92] + y[58] * (1 - x[94]) + y[
                    59] * x[94] \
                + y[60] * (1 - x[62]) + y[61] * x[62] + y[62] * (1 - x[31]) + y[63] * x[31] + 100 * (
                            x[0] + x[1] + x[2] + x[3] + x[4] + x[5] + x[6] + x[8] + x[9] + x[10] + x[12] + x[16] + x[
                        17] + x[18] + x[20] + x[24])
    H = cost_func + constraint_func
    model = H.compile()
    bqm = model.to_bqm()
    sa = neal.SimulatedAnnealingSampler()
    sampleset = sa.sample(bqm, num_reads=300)
    decoded_samples = model.decode_sampleset(sampleset)
    best_sample = min(decoded_samples, key=lambda x: x.energy)
    result = [0, 0]
    for i in range(K):
        if line[i] != str(best_sample.sample[a[index[i]]]):
            result[0] = result[0] + 1
    raw_solution = best_sample.sample
    decoded_sample = model.decode_sample(raw_solution, vartype='BINARY')
    for c in decoded_sample.constraints(only_broken=True):
        if c[0] == 'E':
            result[1] = 1
    q.put(result)


def thr9(y, line, N, K, n_var, a, We, q, index):
    x = [Binary(a[i]) for i in range(n_var)]
    NodeConstraints = []
    NodeConstraints.append(Constraint((x[0] + x[1] + x[32] - 2 * x[33]), label="En"))
    NodeConstraints.append(Constraint((x[2] + x[3] + x[34] - 2 * x[35]), label="En"))
    NodeConstraints.append(Constraint((x[4] + x[5] + x[36] - 2 * x[37]), label="En"))
    NodeConstraints.append(Constraint((x[6] + x[7] + x[38] - 2 * x[39]), label="En"))
    NodeConstraints.append(Constraint((x[8] + x[9] + x[40] - 2 * x[41]), label="En"))
    NodeConstraints.append(Constraint((x[10] + x[11] + x[42] - 2 * x[43]), label="En"))
    NodeConstraints.append(Constraint((x[12] + x[13] + x[44] - 2 * x[45]), label="En"))
    NodeConstraints.append(Constraint((x[14] + x[15] + x[46] - 2 * x[47]), label="En"))
    NodeConstraints.append(Constraint((x[16] + x[17] + x[48] - 2 * x[49]), label="En"))
    NodeConstraints.append(Constraint((x[18] + x[19] + x[50] - 2 * x[51]), label="En"))
    NodeConstraints.append(Constraint((x[20] + x[21] + x[52] - 2 * x[53]), label="En"))
    NodeConstraints.append(Constraint((x[22] + x[23] + x[54] - 2 * x[55]), label="En"))
    NodeConstraints.append(Constraint((x[24] + x[25] + x[56] - 2 * x[57]), label="En"))
    NodeConstraints.append(Constraint((x[26] + x[27] + x[58] - 2 * x[59]), label="En"))
    NodeConstraints.append(Constraint((x[28] + x[29] + x[60] - 2 * x[61]), label="En"))
    NodeConstraints.append(Constraint((x[30] + x[31] + x[62] - 2 * x[63]), label="En"))
    NodeConstraints.append(Constraint((x[32] + x[34] + x[64] - 2 * x[65]), label="En"))
    NodeConstraints.append(Constraint((x[1] + x[3] + x[66] - 2 * x[67]), label="En"))
    NodeConstraints.append(Constraint((x[36] + x[38] + x[68] - 2 * x[69]), label="En"))
    NodeConstraints.append(Constraint((x[5] + x[7] + x[70] - 2 * x[71]), label="En"))
    NodeConstraints.append(Constraint((x[40] + x[42] + x[72] - 2 * x[73]), label="En"))
    NodeConstraints.append(Constraint((x[9] + x[11] + x[74] - 2 * x[75]), label="En"))
    NodeConstraints.append(Constraint((x[44] + x[46] + x[76] - 2 * x[77]), label="En"))
    NodeConstraints.append(Constraint((x[13] + x[15] + x[78] - 2 * x[79]), label="En"))
    NodeConstraints.append(Constraint((x[48] + x[50] + x[80] - 2 * x[81]), label="En"))
    NodeConstraints.append(Constraint((x[17] + x[19] + x[82] - 2 * x[83]), label="En"))
    NodeConstraints.append(Constraint((x[52] + x[54] + x[84] - 2 * x[85]), label="En"))
    NodeConstraints.append(Constraint((x[21] + x[23] + x[86] - 2 * x[87]), label="En"))
    NodeConstraints.append(Constraint((x[56] + x[58] + x[88] - 2 * x[89]), label="En"))
    NodeConstraints.append(Constraint((x[25] + x[27] + x[90] - 2 * x[91]), label="En"))
    NodeConstraints.append(Constraint((x[60] + x[62] + x[92] - 2 * x[93]), label="En"))
    NodeConstraints.append(Constraint((x[29] + x[31] + x[94] - 2 * x[95]), label="En"))
    NodeConstraints.append(Constraint((x[64] + x[68] + x[96] - 2 * x[97]), label="En"))
    NodeConstraints.append(Constraint((x[66] + x[70] + x[98] - 2 * x[99]), label="En"))
    NodeConstraints.append(Constraint((x[34] + x[38] + x[100] - 2 * x[101]), label="En"))
    NodeConstraints.append(Constraint((x[3] + x[7] + x[102] - 2 * x[103]), label="En"))
    NodeConstraints.append(Constraint((x[72] + x[76] + x[104] - 2 * x[105]), label="En"))
    NodeConstraints.append(Constraint((x[74] + x[78] + x[106] - 2 * x[107]), label="En"))
    NodeConstraints.append(Constraint((x[42] + x[46] + x[108] - 2 * x[109]), label="En"))
    NodeConstraints.append(Constraint((x[11] + x[15] + x[110] - 2 * x[111]), label="En"))
    NodeConstraints.append(Constraint((x[80] + x[84] + x[112] - 2 * x[113]), label="En"))
    NodeConstraints.append(Constraint((x[82] + x[86] + x[114] - 2 * x[115]), label="En"))
    NodeConstraints.append(Constraint((x[50] + x[54] + x[116] - 2 * x[117]), label="En"))
    NodeConstraints.append(Constraint((x[19] + x[23] + x[118] - 2 * x[119]), label="En"))
    NodeConstraints.append(Constraint((x[88] + x[92] + x[120] - 2 * x[121]), label="En"))
    NodeConstraints.append(Constraint((x[90] + x[94] + x[122] - 2 * x[123]), label="En"))
    NodeConstraints.append(Constraint((x[58] + x[62] + x[124] - 2 * x[125]), label="En"))
    NodeConstraints.append(Constraint((x[27] + x[31] + x[126] - 2 * x[127]), label="En"))
    NodeConstraints.append(Constraint((x[96] + x[104] + x[128] - 2 * x[129]), label="En"))
    NodeConstraints.append(Constraint((x[98] + x[106] + x[130] - 2 * x[131]), label="En"))
    NodeConstraints.append(Constraint((x[100] + x[108] + x[132] - 2 * x[133]), label="En"))
    NodeConstraints.append(Constraint((x[102] + x[110] + x[134] - 2 * x[135]), label="En"))
    NodeConstraints.append(Constraint((x[68] + x[76] + x[136] - 2 * x[137]), label="En"))
    NodeConstraints.append(Constraint((x[70] + x[78] + x[138] - 2 * x[139]), label="En"))
    NodeConstraints.append(Constraint((x[38] + x[46] + x[140] - 2 * x[141]), label="En"))
    NodeConstraints.append(Constraint((x[7] + x[15] + x[142] - 2 * x[143]), label="En"))
    NodeConstraints.append(Constraint((x[112] + x[120] + x[144] - 2 * x[145]), label="En"))
    NodeConstraints.append(Constraint((x[114] + x[122] + x[146] - 2 * x[147]), label="En"))
    NodeConstraints.append(Constraint((x[116] + x[124] + x[148] - 2 * x[149]), label="En"))
    NodeConstraints.append(Constraint((x[118] + x[126] + x[150] - 2 * x[151]), label="En"))
    NodeConstraints.append(Constraint((x[84] + x[92] + x[152] - 2 * x[153]), label="En"))
    NodeConstraints.append(Constraint((x[86] + x[94] + x[154] - 2 * x[155]), label="En"))
    NodeConstraints.append(Constraint((x[54] + x[62] + x[156] - 2 * x[157]), label="En"))
    NodeConstraints.append(Constraint((x[23] + x[31] + x[158] - 2 * x[159]), label="En"))
    NodeConstraints.append(Constraint((x[128] + x[144] + x[160] - 2 * x[161]), label="En"))
    NodeConstraints.append(Constraint((x[130] + x[146] + x[162] - 2 * x[163]), label="En"))
    NodeConstraints.append(Constraint((x[132] + x[148] + x[164] - 2 * x[165]), label="En"))
    NodeConstraints.append(Constraint((x[134] + x[150] + x[166] - 2 * x[167]), label="En"))
    NodeConstraints.append(Constraint((x[136] + x[152] + x[168] - 2 * x[169]), label="En"))
    NodeConstraints.append(Constraint((x[138] + x[154] + x[170] - 2 * x[171]), label="En"))
    NodeConstraints.append(Constraint((x[140] + x[156] + x[172] - 2 * x[173]), label="En"))
    NodeConstraints.append(Constraint((x[142] + x[158] + x[174] - 2 * x[175]), label="En"))
    NodeConstraints.append(Constraint((x[104] + x[120] + x[176] - 2 * x[177]), label="En"))
    NodeConstraints.append(Constraint((x[106] + x[122] + x[178] - 2 * x[179]), label="En"))
    NodeConstraints.append(Constraint((x[108] + x[124] + x[180] - 2 * x[181]), label="En"))
    NodeConstraints.append(Constraint((x[110] + x[126] + x[182] - 2 * x[183]), label="En"))
    NodeConstraints.append(Constraint((x[76] + x[92] + x[184] - 2 * x[185]), label="En"))
    NodeConstraints.append(Constraint((x[78] + x[94] + x[186] - 2 * x[187]), label="En"))
    NodeConstraints.append(Constraint((x[46] + x[62] + x[188] - 2 * x[189]), label="En"))
    NodeConstraints.append(Constraint((x[15] + x[31] + x[190] - 2 * x[191]), label="En"))
    NodeConstraints = [c * c for c in NodeConstraints]
    constraint_func = We * sum(NodeConstraints)
    cost_func = y[0] * (1 - x[160]) + y[1] * x[160] + y[2] * (1 - x[162]) + y[3] * x[162] + y[4] * (1 - x[164]) + y[5] * \
                x[164] \
                + y[6] * (1 - x[166]) + y[7] * x[166] + y[8] * (1 - x[168]) + y[9] * x[168] + y[10] * (1 - x[170]) + y[
                    11] * x[170] \
                + y[12] * (1 - x[172]) + y[13] * x[172] + y[14] * (1 - x[174]) + y[15] * x[174] + y[16] * (1 - x[176]) + \
                y[17] * x[176] \
                + y[18] * (1 - x[178]) + y[19] * x[178] + y[20] * (1 - x[180]) + y[21] * x[180] + y[22] * (1 - x[182]) + \
                y[23] * x[182] \
                + y[24] * (1 - x[184]) + y[25] * x[184] + y[26] * (1 - x[186]) + y[27] * x[186] + y[28] * (1 - x[188]) + \
                y[29] * x[188] \
                + y[30] * (1 - x[190]) + y[31] * x[190] + y[32] * (1 - x[144]) + y[33] * x[144] + y[34] * (1 - x[146]) + \
                y[35] * x[146] \
                + y[36] * (1 - x[148]) + y[37] * x[148] + y[38] * (1 - x[150]) + y[39] * x[150] + y[40] * (1 - x[152]) + \
                y[41] * x[152] \
                + y[42] * (1 - x[154]) + y[43] * x[154] + y[44] * (1 - x[156]) + y[45] * x[156] + y[46] * (1 - x[158]) + \
                y[47] * x[158] \
                + y[48] * (1 - x[120]) + y[49] * x[120] + y[50] * (1 - x[122]) + y[51] * x[122] + y[52] * (1 - x[124]) + \
                y[53] * x[124] \
                + y[54] * (1 - x[126]) + y[55] * x[126] + y[56] * (1 - x[92]) + y[57] * x[92] + y[58] * (1 - x[94]) + y[
                    59] * x[94] \
                + y[60] * (1 - x[62]) + y[61] * x[62] + y[62] * (1 - x[31]) + y[63] * x[31] + 100 * (
                            x[0] + x[1] + x[2] + x[3] + x[4] + x[5] + x[6] + x[8] + x[9] + x[10] + x[12] + x[16] + x[
                        17] + x[18] + x[20] + x[24])
    H = cost_func + constraint_func
    model = H.compile()
    bqm = model.to_bqm()
    sa = neal.SimulatedAnnealingSampler()
    sampleset = sa.sample(bqm, num_reads=300)
    decoded_samples = model.decode_sampleset(sampleset)
    best_sample = min(decoded_samples, key=lambda x: x.energy)
    result = [0, 0]
    for i in range(K):
        if line[i] != str(best_sample.sample[a[index[i]]]):
            result[0] = result[0] + 1
    raw_solution = best_sample.sample
    decoded_sample = model.decode_sample(raw_solution, vartype='BINARY')
    for c in decoded_sample.constraints(only_broken=True):
        if c[0] == 'E':
            result[1] = 1
    q.put(result)


def thr10(y, line, N, K, n_var, a, We, q, index):
    x = [Binary(a[i]) for i in range(n_var)]
    NodeConstraints = []
    NodeConstraints.append(Constraint((x[0] + x[1] + x[32] - 2 * x[33]), label="En"))
    NodeConstraints.append(Constraint((x[2] + x[3] + x[34] - 2 * x[35]), label="En"))
    NodeConstraints.append(Constraint((x[4] + x[5] + x[36] - 2 * x[37]), label="En"))
    NodeConstraints.append(Constraint((x[6] + x[7] + x[38] - 2 * x[39]), label="En"))
    NodeConstraints.append(Constraint((x[8] + x[9] + x[40] - 2 * x[41]), label="En"))
    NodeConstraints.append(Constraint((x[10] + x[11] + x[42] - 2 * x[43]), label="En"))
    NodeConstraints.append(Constraint((x[12] + x[13] + x[44] - 2 * x[45]), label="En"))
    NodeConstraints.append(Constraint((x[14] + x[15] + x[46] - 2 * x[47]), label="En"))
    NodeConstraints.append(Constraint((x[16] + x[17] + x[48] - 2 * x[49]), label="En"))
    NodeConstraints.append(Constraint((x[18] + x[19] + x[50] - 2 * x[51]), label="En"))
    NodeConstraints.append(Constraint((x[20] + x[21] + x[52] - 2 * x[53]), label="En"))
    NodeConstraints.append(Constraint((x[22] + x[23] + x[54] - 2 * x[55]), label="En"))
    NodeConstraints.append(Constraint((x[24] + x[25] + x[56] - 2 * x[57]), label="En"))
    NodeConstraints.append(Constraint((x[26] + x[27] + x[58] - 2 * x[59]), label="En"))
    NodeConstraints.append(Constraint((x[28] + x[29] + x[60] - 2 * x[61]), label="En"))
    NodeConstraints.append(Constraint((x[30] + x[31] + x[62] - 2 * x[63]), label="En"))
    NodeConstraints.append(Constraint((x[32] + x[34] + x[64] - 2 * x[65]), label="En"))
    NodeConstraints.append(Constraint((x[1] + x[3] + x[66] - 2 * x[67]), label="En"))
    NodeConstraints.append(Constraint((x[36] + x[38] + x[68] - 2 * x[69]), label="En"))
    NodeConstraints.append(Constraint((x[5] + x[7] + x[70] - 2 * x[71]), label="En"))
    NodeConstraints.append(Constraint((x[40] + x[42] + x[72] - 2 * x[73]), label="En"))
    NodeConstraints.append(Constraint((x[9] + x[11] + x[74] - 2 * x[75]), label="En"))
    NodeConstraints.append(Constraint((x[44] + x[46] + x[76] - 2 * x[77]), label="En"))
    NodeConstraints.append(Constraint((x[13] + x[15] + x[78] - 2 * x[79]), label="En"))
    NodeConstraints.append(Constraint((x[48] + x[50] + x[80] - 2 * x[81]), label="En"))
    NodeConstraints.append(Constraint((x[17] + x[19] + x[82] - 2 * x[83]), label="En"))
    NodeConstraints.append(Constraint((x[52] + x[54] + x[84] - 2 * x[85]), label="En"))
    NodeConstraints.append(Constraint((x[21] + x[23] + x[86] - 2 * x[87]), label="En"))
    NodeConstraints.append(Constraint((x[56] + x[58] + x[88] - 2 * x[89]), label="En"))
    NodeConstraints.append(Constraint((x[25] + x[27] + x[90] - 2 * x[91]), label="En"))
    NodeConstraints.append(Constraint((x[60] + x[62] + x[92] - 2 * x[93]), label="En"))
    NodeConstraints.append(Constraint((x[29] + x[31] + x[94] - 2 * x[95]), label="En"))
    NodeConstraints.append(Constraint((x[64] + x[68] + x[96] - 2 * x[97]), label="En"))
    NodeConstraints.append(Constraint((x[66] + x[70] + x[98] - 2 * x[99]), label="En"))
    NodeConstraints.append(Constraint((x[34] + x[38] + x[100] - 2 * x[101]), label="En"))
    NodeConstraints.append(Constraint((x[3] + x[7] + x[102] - 2 * x[103]), label="En"))
    NodeConstraints.append(Constraint((x[72] + x[76] + x[104] - 2 * x[105]), label="En"))
    NodeConstraints.append(Constraint((x[74] + x[78] + x[106] - 2 * x[107]), label="En"))
    NodeConstraints.append(Constraint((x[42] + x[46] + x[108] - 2 * x[109]), label="En"))
    NodeConstraints.append(Constraint((x[11] + x[15] + x[110] - 2 * x[111]), label="En"))
    NodeConstraints.append(Constraint((x[80] + x[84] + x[112] - 2 * x[113]), label="En"))
    NodeConstraints.append(Constraint((x[82] + x[86] + x[114] - 2 * x[115]), label="En"))
    NodeConstraints.append(Constraint((x[50] + x[54] + x[116] - 2 * x[117]), label="En"))
    NodeConstraints.append(Constraint((x[19] + x[23] + x[118] - 2 * x[119]), label="En"))
    NodeConstraints.append(Constraint((x[88] + x[92] + x[120] - 2 * x[121]), label="En"))
    NodeConstraints.append(Constraint((x[90] + x[94] + x[122] - 2 * x[123]), label="En"))
    NodeConstraints.append(Constraint((x[58] + x[62] + x[124] - 2 * x[125]), label="En"))
    NodeConstraints.append(Constraint((x[27] + x[31] + x[126] - 2 * x[127]), label="En"))
    NodeConstraints.append(Constraint((x[96] + x[104] + x[128] - 2 * x[129]), label="En"))
    NodeConstraints.append(Constraint((x[98] + x[106] + x[130] - 2 * x[131]), label="En"))
    NodeConstraints.append(Constraint((x[100] + x[108] + x[132] - 2 * x[133]), label="En"))
    NodeConstraints.append(Constraint((x[102] + x[110] + x[134] - 2 * x[135]), label="En"))
    NodeConstraints.append(Constraint((x[68] + x[76] + x[136] - 2 * x[137]), label="En"))
    NodeConstraints.append(Constraint((x[70] + x[78] + x[138] - 2 * x[139]), label="En"))
    NodeConstraints.append(Constraint((x[38] + x[46] + x[140] - 2 * x[141]), label="En"))
    NodeConstraints.append(Constraint((x[7] + x[15] + x[142] - 2 * x[143]), label="En"))
    NodeConstraints.append(Constraint((x[112] + x[120] + x[144] - 2 * x[145]), label="En"))
    NodeConstraints.append(Constraint((x[114] + x[122] + x[146] - 2 * x[147]), label="En"))
    NodeConstraints.append(Constraint((x[116] + x[124] + x[148] - 2 * x[149]), label="En"))
    NodeConstraints.append(Constraint((x[118] + x[126] + x[150] - 2 * x[151]), label="En"))
    NodeConstraints.append(Constraint((x[84] + x[92] + x[152] - 2 * x[153]), label="En"))
    NodeConstraints.append(Constraint((x[86] + x[94] + x[154] - 2 * x[155]), label="En"))
    NodeConstraints.append(Constraint((x[54] + x[62] + x[156] - 2 * x[157]), label="En"))
    NodeConstraints.append(Constraint((x[23] + x[31] + x[158] - 2 * x[159]), label="En"))
    NodeConstraints.append(Constraint((x[128] + x[144] + x[160] - 2 * x[161]), label="En"))
    NodeConstraints.append(Constraint((x[130] + x[146] + x[162] - 2 * x[163]), label="En"))
    NodeConstraints.append(Constraint((x[132] + x[148] + x[164] - 2 * x[165]), label="En"))
    NodeConstraints.append(Constraint((x[134] + x[150] + x[166] - 2 * x[167]), label="En"))
    NodeConstraints.append(Constraint((x[136] + x[152] + x[168] - 2 * x[169]), label="En"))
    NodeConstraints.append(Constraint((x[138] + x[154] + x[170] - 2 * x[171]), label="En"))
    NodeConstraints.append(Constraint((x[140] + x[156] + x[172] - 2 * x[173]), label="En"))
    NodeConstraints.append(Constraint((x[142] + x[158] + x[174] - 2 * x[175]), label="En"))
    NodeConstraints.append(Constraint((x[104] + x[120] + x[176] - 2 * x[177]), label="En"))
    NodeConstraints.append(Constraint((x[106] + x[122] + x[178] - 2 * x[179]), label="En"))
    NodeConstraints.append(Constraint((x[108] + x[124] + x[180] - 2 * x[181]), label="En"))
    NodeConstraints.append(Constraint((x[110] + x[126] + x[182] - 2 * x[183]), label="En"))
    NodeConstraints.append(Constraint((x[76] + x[92] + x[184] - 2 * x[185]), label="En"))
    NodeConstraints.append(Constraint((x[78] + x[94] + x[186] - 2 * x[187]), label="En"))
    NodeConstraints.append(Constraint((x[46] + x[62] + x[188] - 2 * x[189]), label="En"))
    NodeConstraints.append(Constraint((x[15] + x[31] + x[190] - 2 * x[191]), label="En"))
    NodeConstraints = [c * c for c in NodeConstraints]
    constraint_func = We * sum(NodeConstraints)
    cost_func = y[0] * (1 - x[160]) + y[1] * x[160] + y[2] * (1 - x[162]) + y[3] * x[162] + y[4] * (1 - x[164]) + y[5] * \
                x[164] \
                + y[6] * (1 - x[166]) + y[7] * x[166] + y[8] * (1 - x[168]) + y[9] * x[168] + y[10] * (1 - x[170]) + y[
                    11] * x[170] \
                + y[12] * (1 - x[172]) + y[13] * x[172] + y[14] * (1 - x[174]) + y[15] * x[174] + y[16] * (1 - x[176]) + \
                y[17] * x[176] \
                + y[18] * (1 - x[178]) + y[19] * x[178] + y[20] * (1 - x[180]) + y[21] * x[180] + y[22] * (1 - x[182]) + \
                y[23] * x[182] \
                + y[24] * (1 - x[184]) + y[25] * x[184] + y[26] * (1 - x[186]) + y[27] * x[186] + y[28] * (1 - x[188]) + \
                y[29] * x[188] \
                + y[30] * (1 - x[190]) + y[31] * x[190] + y[32] * (1 - x[144]) + y[33] * x[144] + y[34] * (1 - x[146]) + \
                y[35] * x[146] \
                + y[36] * (1 - x[148]) + y[37] * x[148] + y[38] * (1 - x[150]) + y[39] * x[150] + y[40] * (1 - x[152]) + \
                y[41] * x[152] \
                + y[42] * (1 - x[154]) + y[43] * x[154] + y[44] * (1 - x[156]) + y[45] * x[156] + y[46] * (1 - x[158]) + \
                y[47] * x[158] \
                + y[48] * (1 - x[120]) + y[49] * x[120] + y[50] * (1 - x[122]) + y[51] * x[122] + y[52] * (1 - x[124]) + \
                y[53] * x[124] \
                + y[54] * (1 - x[126]) + y[55] * x[126] + y[56] * (1 - x[92]) + y[57] * x[92] + y[58] * (1 - x[94]) + y[
                    59] * x[94] \
                + y[60] * (1 - x[62]) + y[61] * x[62] + y[62] * (1 - x[31]) + y[63] * x[31] + 100 * (
                            x[0] + x[1] + x[2] + x[3] + x[4] + x[5] + x[6] + x[8] + x[9] + x[10] + x[12] + x[16] + x[
                        17] + x[18] + x[20] + x[24])
    H = cost_func + constraint_func
    model = H.compile()
    bqm = model.to_bqm()
    sa = neal.SimulatedAnnealingSampler()
    sampleset = sa.sample(bqm, num_reads=300)
    decoded_samples = model.decode_sampleset(sampleset)
    best_sample = min(decoded_samples, key=lambda x: x.energy)
    result = [0, 0]
    for i in range(K):
        if line[i] != str(best_sample.sample[a[index[i]]]):
            result[0] = result[0] + 1
    raw_solution = best_sample.sample
    decoded_sample = model.decode_sample(raw_solution, vartype='BINARY')
    for c in decoded_sample.constraints(only_broken=True):
        if c[0] == 'E':
            result[1] = 1
    q.put(result)


def thr11(y, line, N, K, n_var, a, We, q, index):
    x = [Binary(a[i]) for i in range(n_var)]
    NodeConstraints = []
    NodeConstraints.append(Constraint((x[0] + x[1] + x[32] - 2 * x[33]), label="En"))
    NodeConstraints.append(Constraint((x[2] + x[3] + x[34] - 2 * x[35]), label="En"))
    NodeConstraints.append(Constraint((x[4] + x[5] + x[36] - 2 * x[37]), label="En"))
    NodeConstraints.append(Constraint((x[6] + x[7] + x[38] - 2 * x[39]), label="En"))
    NodeConstraints.append(Constraint((x[8] + x[9] + x[40] - 2 * x[41]), label="En"))
    NodeConstraints.append(Constraint((x[10] + x[11] + x[42] - 2 * x[43]), label="En"))
    NodeConstraints.append(Constraint((x[12] + x[13] + x[44] - 2 * x[45]), label="En"))
    NodeConstraints.append(Constraint((x[14] + x[15] + x[46] - 2 * x[47]), label="En"))
    NodeConstraints.append(Constraint((x[16] + x[17] + x[48] - 2 * x[49]), label="En"))
    NodeConstraints.append(Constraint((x[18] + x[19] + x[50] - 2 * x[51]), label="En"))
    NodeConstraints.append(Constraint((x[20] + x[21] + x[52] - 2 * x[53]), label="En"))
    NodeConstraints.append(Constraint((x[22] + x[23] + x[54] - 2 * x[55]), label="En"))
    NodeConstraints.append(Constraint((x[24] + x[25] + x[56] - 2 * x[57]), label="En"))
    NodeConstraints.append(Constraint((x[26] + x[27] + x[58] - 2 * x[59]), label="En"))
    NodeConstraints.append(Constraint((x[28] + x[29] + x[60] - 2 * x[61]), label="En"))
    NodeConstraints.append(Constraint((x[30] + x[31] + x[62] - 2 * x[63]), label="En"))
    NodeConstraints.append(Constraint((x[32] + x[34] + x[64] - 2 * x[65]), label="En"))
    NodeConstraints.append(Constraint((x[1] + x[3] + x[66] - 2 * x[67]), label="En"))
    NodeConstraints.append(Constraint((x[36] + x[38] + x[68] - 2 * x[69]), label="En"))
    NodeConstraints.append(Constraint((x[5] + x[7] + x[70] - 2 * x[71]), label="En"))
    NodeConstraints.append(Constraint((x[40] + x[42] + x[72] - 2 * x[73]), label="En"))
    NodeConstraints.append(Constraint((x[9] + x[11] + x[74] - 2 * x[75]), label="En"))
    NodeConstraints.append(Constraint((x[44] + x[46] + x[76] - 2 * x[77]), label="En"))
    NodeConstraints.append(Constraint((x[13] + x[15] + x[78] - 2 * x[79]), label="En"))
    NodeConstraints.append(Constraint((x[48] + x[50] + x[80] - 2 * x[81]), label="En"))
    NodeConstraints.append(Constraint((x[17] + x[19] + x[82] - 2 * x[83]), label="En"))
    NodeConstraints.append(Constraint((x[52] + x[54] + x[84] - 2 * x[85]), label="En"))
    NodeConstraints.append(Constraint((x[21] + x[23] + x[86] - 2 * x[87]), label="En"))
    NodeConstraints.append(Constraint((x[56] + x[58] + x[88] - 2 * x[89]), label="En"))
    NodeConstraints.append(Constraint((x[25] + x[27] + x[90] - 2 * x[91]), label="En"))
    NodeConstraints.append(Constraint((x[60] + x[62] + x[92] - 2 * x[93]), label="En"))
    NodeConstraints.append(Constraint((x[29] + x[31] + x[94] - 2 * x[95]), label="En"))
    NodeConstraints.append(Constraint((x[64] + x[68] + x[96] - 2 * x[97]), label="En"))
    NodeConstraints.append(Constraint((x[66] + x[70] + x[98] - 2 * x[99]), label="En"))
    NodeConstraints.append(Constraint((x[34] + x[38] + x[100] - 2 * x[101]), label="En"))
    NodeConstraints.append(Constraint((x[3] + x[7] + x[102] - 2 * x[103]), label="En"))
    NodeConstraints.append(Constraint((x[72] + x[76] + x[104] - 2 * x[105]), label="En"))
    NodeConstraints.append(Constraint((x[74] + x[78] + x[106] - 2 * x[107]), label="En"))
    NodeConstraints.append(Constraint((x[42] + x[46] + x[108] - 2 * x[109]), label="En"))
    NodeConstraints.append(Constraint((x[11] + x[15] + x[110] - 2 * x[111]), label="En"))
    NodeConstraints.append(Constraint((x[80] + x[84] + x[112] - 2 * x[113]), label="En"))
    NodeConstraints.append(Constraint((x[82] + x[86] + x[114] - 2 * x[115]), label="En"))
    NodeConstraints.append(Constraint((x[50] + x[54] + x[116] - 2 * x[117]), label="En"))
    NodeConstraints.append(Constraint((x[19] + x[23] + x[118] - 2 * x[119]), label="En"))
    NodeConstraints.append(Constraint((x[88] + x[92] + x[120] - 2 * x[121]), label="En"))
    NodeConstraints.append(Constraint((x[90] + x[94] + x[122] - 2 * x[123]), label="En"))
    NodeConstraints.append(Constraint((x[58] + x[62] + x[124] - 2 * x[125]), label="En"))
    NodeConstraints.append(Constraint((x[27] + x[31] + x[126] - 2 * x[127]), label="En"))
    NodeConstraints.append(Constraint((x[96] + x[104] + x[128] - 2 * x[129]), label="En"))
    NodeConstraints.append(Constraint((x[98] + x[106] + x[130] - 2 * x[131]), label="En"))
    NodeConstraints.append(Constraint((x[100] + x[108] + x[132] - 2 * x[133]), label="En"))
    NodeConstraints.append(Constraint((x[102] + x[110] + x[134] - 2 * x[135]), label="En"))
    NodeConstraints.append(Constraint((x[68] + x[76] + x[136] - 2 * x[137]), label="En"))
    NodeConstraints.append(Constraint((x[70] + x[78] + x[138] - 2 * x[139]), label="En"))
    NodeConstraints.append(Constraint((x[38] + x[46] + x[140] - 2 * x[141]), label="En"))
    NodeConstraints.append(Constraint((x[7] + x[15] + x[142] - 2 * x[143]), label="En"))
    NodeConstraints.append(Constraint((x[112] + x[120] + x[144] - 2 * x[145]), label="En"))
    NodeConstraints.append(Constraint((x[114] + x[122] + x[146] - 2 * x[147]), label="En"))
    NodeConstraints.append(Constraint((x[116] + x[124] + x[148] - 2 * x[149]), label="En"))
    NodeConstraints.append(Constraint((x[118] + x[126] + x[150] - 2 * x[151]), label="En"))
    NodeConstraints.append(Constraint((x[84] + x[92] + x[152] - 2 * x[153]), label="En"))
    NodeConstraints.append(Constraint((x[86] + x[94] + x[154] - 2 * x[155]), label="En"))
    NodeConstraints.append(Constraint((x[54] + x[62] + x[156] - 2 * x[157]), label="En"))
    NodeConstraints.append(Constraint((x[23] + x[31] + x[158] - 2 * x[159]), label="En"))
    NodeConstraints.append(Constraint((x[128] + x[144] + x[160] - 2 * x[161]), label="En"))
    NodeConstraints.append(Constraint((x[130] + x[146] + x[162] - 2 * x[163]), label="En"))
    NodeConstraints.append(Constraint((x[132] + x[148] + x[164] - 2 * x[165]), label="En"))
    NodeConstraints.append(Constraint((x[134] + x[150] + x[166] - 2 * x[167]), label="En"))
    NodeConstraints.append(Constraint((x[136] + x[152] + x[168] - 2 * x[169]), label="En"))
    NodeConstraints.append(Constraint((x[138] + x[154] + x[170] - 2 * x[171]), label="En"))
    NodeConstraints.append(Constraint((x[140] + x[156] + x[172] - 2 * x[173]), label="En"))
    NodeConstraints.append(Constraint((x[142] + x[158] + x[174] - 2 * x[175]), label="En"))
    NodeConstraints.append(Constraint((x[104] + x[120] + x[176] - 2 * x[177]), label="En"))
    NodeConstraints.append(Constraint((x[106] + x[122] + x[178] - 2 * x[179]), label="En"))
    NodeConstraints.append(Constraint((x[108] + x[124] + x[180] - 2 * x[181]), label="En"))
    NodeConstraints.append(Constraint((x[110] + x[126] + x[182] - 2 * x[183]), label="En"))
    NodeConstraints.append(Constraint((x[76] + x[92] + x[184] - 2 * x[185]), label="En"))
    NodeConstraints.append(Constraint((x[78] + x[94] + x[186] - 2 * x[187]), label="En"))
    NodeConstraints.append(Constraint((x[46] + x[62] + x[188] - 2 * x[189]), label="En"))
    NodeConstraints.append(Constraint((x[15] + x[31] + x[190] - 2 * x[191]), label="En"))
    NodeConstraints = [c * c for c in NodeConstraints]
    constraint_func = We * sum(NodeConstraints)
    cost_func = y[0] * (1 - x[160]) + y[1] * x[160] + y[2] * (1 - x[162]) + y[3] * x[162] + y[4] * (1 - x[164]) + y[5] * \
                x[164] \
                + y[6] * (1 - x[166]) + y[7] * x[166] + y[8] * (1 - x[168]) + y[9] * x[168] + y[10] * (1 - x[170]) + y[
                    11] * x[170] \
                + y[12] * (1 - x[172]) + y[13] * x[172] + y[14] * (1 - x[174]) + y[15] * x[174] + y[16] * (1 - x[176]) + \
                y[17] * x[176] \
                + y[18] * (1 - x[178]) + y[19] * x[178] + y[20] * (1 - x[180]) + y[21] * x[180] + y[22] * (1 - x[182]) + \
                y[23] * x[182] \
                + y[24] * (1 - x[184]) + y[25] * x[184] + y[26] * (1 - x[186]) + y[27] * x[186] + y[28] * (1 - x[188]) + \
                y[29] * x[188] \
                + y[30] * (1 - x[190]) + y[31] * x[190] + y[32] * (1 - x[144]) + y[33] * x[144] + y[34] * (1 - x[146]) + \
                y[35] * x[146] \
                + y[36] * (1 - x[148]) + y[37] * x[148] + y[38] * (1 - x[150]) + y[39] * x[150] + y[40] * (1 - x[152]) + \
                y[41] * x[152] \
                + y[42] * (1 - x[154]) + y[43] * x[154] + y[44] * (1 - x[156]) + y[45] * x[156] + y[46] * (1 - x[158]) + \
                y[47] * x[158] \
                + y[48] * (1 - x[120]) + y[49] * x[120] + y[50] * (1 - x[122]) + y[51] * x[122] + y[52] * (1 - x[124]) + \
                y[53] * x[124] \
                + y[54] * (1 - x[126]) + y[55] * x[126] + y[56] * (1 - x[92]) + y[57] * x[92] + y[58] * (1 - x[94]) + y[
                    59] * x[94] \
                + y[60] * (1 - x[62]) + y[61] * x[62] + y[62] * (1 - x[31]) + y[63] * x[31] + 100 * (
                            x[0] + x[1] + x[2] + x[3] + x[4] + x[5] + x[6] + x[8] + x[9] + x[10] + x[12] + x[16] + x[
                        17] + x[18] + x[20] + x[24])
    H = cost_func + constraint_func
    model = H.compile()
    bqm = model.to_bqm()
    sa = neal.SimulatedAnnealingSampler()
    sampleset = sa.sample(bqm, num_reads=300)
    decoded_samples = model.decode_sampleset(sampleset)
    best_sample = min(decoded_samples, key=lambda x: x.energy)
    result = [0, 0]
    for i in range(K):
        if line[i] != str(best_sample.sample[a[index[i]]]):
            result[0] = result[0] + 1
    raw_solution = best_sample.sample
    decoded_sample = model.decode_sample(raw_solution, vartype='BINARY')
    for c in decoded_sample.constraints(only_broken=True):
        if c[0] == 'E':
            result[1] = 1
    q.put(result)


def thr12(y, line, N, K, n_var, a, We, q, index):
    x = [Binary(a[i]) for i in range(n_var)]
    NodeConstraints = []
    NodeConstraints.append(Constraint((x[0] + x[1] + x[32] - 2 * x[33]), label="En"))
    NodeConstraints.append(Constraint((x[2] + x[3] + x[34] - 2 * x[35]), label="En"))
    NodeConstraints.append(Constraint((x[4] + x[5] + x[36] - 2 * x[37]), label="En"))
    NodeConstraints.append(Constraint((x[6] + x[7] + x[38] - 2 * x[39]), label="En"))
    NodeConstraints.append(Constraint((x[8] + x[9] + x[40] - 2 * x[41]), label="En"))
    NodeConstraints.append(Constraint((x[10] + x[11] + x[42] - 2 * x[43]), label="En"))
    NodeConstraints.append(Constraint((x[12] + x[13] + x[44] - 2 * x[45]), label="En"))
    NodeConstraints.append(Constraint((x[14] + x[15] + x[46] - 2 * x[47]), label="En"))
    NodeConstraints.append(Constraint((x[16] + x[17] + x[48] - 2 * x[49]), label="En"))
    NodeConstraints.append(Constraint((x[18] + x[19] + x[50] - 2 * x[51]), label="En"))
    NodeConstraints.append(Constraint((x[20] + x[21] + x[52] - 2 * x[53]), label="En"))
    NodeConstraints.append(Constraint((x[22] + x[23] + x[54] - 2 * x[55]), label="En"))
    NodeConstraints.append(Constraint((x[24] + x[25] + x[56] - 2 * x[57]), label="En"))
    NodeConstraints.append(Constraint((x[26] + x[27] + x[58] - 2 * x[59]), label="En"))
    NodeConstraints.append(Constraint((x[28] + x[29] + x[60] - 2 * x[61]), label="En"))
    NodeConstraints.append(Constraint((x[30] + x[31] + x[62] - 2 * x[63]), label="En"))
    NodeConstraints.append(Constraint((x[32] + x[34] + x[64] - 2 * x[65]), label="En"))
    NodeConstraints.append(Constraint((x[1] + x[3] + x[66] - 2 * x[67]), label="En"))
    NodeConstraints.append(Constraint((x[36] + x[38] + x[68] - 2 * x[69]), label="En"))
    NodeConstraints.append(Constraint((x[5] + x[7] + x[70] - 2 * x[71]), label="En"))
    NodeConstraints.append(Constraint((x[40] + x[42] + x[72] - 2 * x[73]), label="En"))
    NodeConstraints.append(Constraint((x[9] + x[11] + x[74] - 2 * x[75]), label="En"))
    NodeConstraints.append(Constraint((x[44] + x[46] + x[76] - 2 * x[77]), label="En"))
    NodeConstraints.append(Constraint((x[13] + x[15] + x[78] - 2 * x[79]), label="En"))
    NodeConstraints.append(Constraint((x[48] + x[50] + x[80] - 2 * x[81]), label="En"))
    NodeConstraints.append(Constraint((x[17] + x[19] + x[82] - 2 * x[83]), label="En"))
    NodeConstraints.append(Constraint((x[52] + x[54] + x[84] - 2 * x[85]), label="En"))
    NodeConstraints.append(Constraint((x[21] + x[23] + x[86] - 2 * x[87]), label="En"))
    NodeConstraints.append(Constraint((x[56] + x[58] + x[88] - 2 * x[89]), label="En"))
    NodeConstraints.append(Constraint((x[25] + x[27] + x[90] - 2 * x[91]), label="En"))
    NodeConstraints.append(Constraint((x[60] + x[62] + x[92] - 2 * x[93]), label="En"))
    NodeConstraints.append(Constraint((x[29] + x[31] + x[94] - 2 * x[95]), label="En"))
    NodeConstraints.append(Constraint((x[64] + x[68] + x[96] - 2 * x[97]), label="En"))
    NodeConstraints.append(Constraint((x[66] + x[70] + x[98] - 2 * x[99]), label="En"))
    NodeConstraints.append(Constraint((x[34] + x[38] + x[100] - 2 * x[101]), label="En"))
    NodeConstraints.append(Constraint((x[3] + x[7] + x[102] - 2 * x[103]), label="En"))
    NodeConstraints.append(Constraint((x[72] + x[76] + x[104] - 2 * x[105]), label="En"))
    NodeConstraints.append(Constraint((x[74] + x[78] + x[106] - 2 * x[107]), label="En"))
    NodeConstraints.append(Constraint((x[42] + x[46] + x[108] - 2 * x[109]), label="En"))
    NodeConstraints.append(Constraint((x[11] + x[15] + x[110] - 2 * x[111]), label="En"))
    NodeConstraints.append(Constraint((x[80] + x[84] + x[112] - 2 * x[113]), label="En"))
    NodeConstraints.append(Constraint((x[82] + x[86] + x[114] - 2 * x[115]), label="En"))
    NodeConstraints.append(Constraint((x[50] + x[54] + x[116] - 2 * x[117]), label="En"))
    NodeConstraints.append(Constraint((x[19] + x[23] + x[118] - 2 * x[119]), label="En"))
    NodeConstraints.append(Constraint((x[88] + x[92] + x[120] - 2 * x[121]), label="En"))
    NodeConstraints.append(Constraint((x[90] + x[94] + x[122] - 2 * x[123]), label="En"))
    NodeConstraints.append(Constraint((x[58] + x[62] + x[124] - 2 * x[125]), label="En"))
    NodeConstraints.append(Constraint((x[27] + x[31] + x[126] - 2 * x[127]), label="En"))
    NodeConstraints.append(Constraint((x[96] + x[104] + x[128] - 2 * x[129]), label="En"))
    NodeConstraints.append(Constraint((x[98] + x[106] + x[130] - 2 * x[131]), label="En"))
    NodeConstraints.append(Constraint((x[100] + x[108] + x[132] - 2 * x[133]), label="En"))
    NodeConstraints.append(Constraint((x[102] + x[110] + x[134] - 2 * x[135]), label="En"))
    NodeConstraints.append(Constraint((x[68] + x[76] + x[136] - 2 * x[137]), label="En"))
    NodeConstraints.append(Constraint((x[70] + x[78] + x[138] - 2 * x[139]), label="En"))
    NodeConstraints.append(Constraint((x[38] + x[46] + x[140] - 2 * x[141]), label="En"))
    NodeConstraints.append(Constraint((x[7] + x[15] + x[142] - 2 * x[143]), label="En"))
    NodeConstraints.append(Constraint((x[112] + x[120] + x[144] - 2 * x[145]), label="En"))
    NodeConstraints.append(Constraint((x[114] + x[122] + x[146] - 2 * x[147]), label="En"))
    NodeConstraints.append(Constraint((x[116] + x[124] + x[148] - 2 * x[149]), label="En"))
    NodeConstraints.append(Constraint((x[118] + x[126] + x[150] - 2 * x[151]), label="En"))
    NodeConstraints.append(Constraint((x[84] + x[92] + x[152] - 2 * x[153]), label="En"))
    NodeConstraints.append(Constraint((x[86] + x[94] + x[154] - 2 * x[155]), label="En"))
    NodeConstraints.append(Constraint((x[54] + x[62] + x[156] - 2 * x[157]), label="En"))
    NodeConstraints.append(Constraint((x[23] + x[31] + x[158] - 2 * x[159]), label="En"))
    NodeConstraints.append(Constraint((x[128] + x[144] + x[160] - 2 * x[161]), label="En"))
    NodeConstraints.append(Constraint((x[130] + x[146] + x[162] - 2 * x[163]), label="En"))
    NodeConstraints.append(Constraint((x[132] + x[148] + x[164] - 2 * x[165]), label="En"))
    NodeConstraints.append(Constraint((x[134] + x[150] + x[166] - 2 * x[167]), label="En"))
    NodeConstraints.append(Constraint((x[136] + x[152] + x[168] - 2 * x[169]), label="En"))
    NodeConstraints.append(Constraint((x[138] + x[154] + x[170] - 2 * x[171]), label="En"))
    NodeConstraints.append(Constraint((x[140] + x[156] + x[172] - 2 * x[173]), label="En"))
    NodeConstraints.append(Constraint((x[142] + x[158] + x[174] - 2 * x[175]), label="En"))
    NodeConstraints.append(Constraint((x[104] + x[120] + x[176] - 2 * x[177]), label="En"))
    NodeConstraints.append(Constraint((x[106] + x[122] + x[178] - 2 * x[179]), label="En"))
    NodeConstraints.append(Constraint((x[108] + x[124] + x[180] - 2 * x[181]), label="En"))
    NodeConstraints.append(Constraint((x[110] + x[126] + x[182] - 2 * x[183]), label="En"))
    NodeConstraints.append(Constraint((x[76] + x[92] + x[184] - 2 * x[185]), label="En"))
    NodeConstraints.append(Constraint((x[78] + x[94] + x[186] - 2 * x[187]), label="En"))
    NodeConstraints.append(Constraint((x[46] + x[62] + x[188] - 2 * x[189]), label="En"))
    NodeConstraints.append(Constraint((x[15] + x[31] + x[190] - 2 * x[191]), label="En"))
    NodeConstraints = [c * c for c in NodeConstraints]
    constraint_func = We * sum(NodeConstraints)
    cost_func = y[0] * (1 - x[160]) + y[1] * x[160] + y[2] * (1 - x[162]) + y[3] * x[162] + y[4] * (1 - x[164]) + y[5] * \
                x[164] \
                + y[6] * (1 - x[166]) + y[7] * x[166] + y[8] * (1 - x[168]) + y[9] * x[168] + y[10] * (1 - x[170]) + y[
                    11] * x[170] \
                + y[12] * (1 - x[172]) + y[13] * x[172] + y[14] * (1 - x[174]) + y[15] * x[174] + y[16] * (1 - x[176]) + \
                y[17] * x[176] \
                + y[18] * (1 - x[178]) + y[19] * x[178] + y[20] * (1 - x[180]) + y[21] * x[180] + y[22] * (1 - x[182]) + \
                y[23] * x[182] \
                + y[24] * (1 - x[184]) + y[25] * x[184] + y[26] * (1 - x[186]) + y[27] * x[186] + y[28] * (1 - x[188]) + \
                y[29] * x[188] \
                + y[30] * (1 - x[190]) + y[31] * x[190] + y[32] * (1 - x[144]) + y[33] * x[144] + y[34] * (1 - x[146]) + \
                y[35] * x[146] \
                + y[36] * (1 - x[148]) + y[37] * x[148] + y[38] * (1 - x[150]) + y[39] * x[150] + y[40] * (1 - x[152]) + \
                y[41] * x[152] \
                + y[42] * (1 - x[154]) + y[43] * x[154] + y[44] * (1 - x[156]) + y[45] * x[156] + y[46] * (1 - x[158]) + \
                y[47] * x[158] \
                + y[48] * (1 - x[120]) + y[49] * x[120] + y[50] * (1 - x[122]) + y[51] * x[122] + y[52] * (1 - x[124]) + \
                y[53] * x[124] \
                + y[54] * (1 - x[126]) + y[55] * x[126] + y[56] * (1 - x[92]) + y[57] * x[92] + y[58] * (1 - x[94]) + y[
                    59] * x[94] \
                + y[60] * (1 - x[62]) + y[61] * x[62] + y[62] * (1 - x[31]) + y[63] * x[31] + 100 * (
                            x[0] + x[1] + x[2] + x[3] + x[4] + x[5] + x[6] + x[8] + x[9] + x[10] + x[12] + x[16] + x[
                        17] + x[18] + x[20] + x[24])
    H = cost_func + constraint_func
    model = H.compile()
    bqm = model.to_bqm()
    sa = neal.SimulatedAnnealingSampler()
    sampleset = sa.sample(bqm, num_reads=300)
    decoded_samples = model.decode_sampleset(sampleset)
    best_sample = min(decoded_samples, key=lambda x: x.energy)
    result = [0, 0]
    for i in range(K):
        if line[i] != str(best_sample.sample[a[index[i]]]):
            result[0] = result[0] + 1
    raw_solution = best_sample.sample
    decoded_sample = model.decode_sample(raw_solution, vartype='BINARY')
    for c in decoded_sample.constraints(only_broken=True):
        if c[0] == 'E':
            result[1] = 1
    q.put(result)


def thr13(y, line, N, K, n_var, a, We, q, index):
    x = [Binary(a[i]) for i in range(n_var)]
    NodeConstraints = []
    NodeConstraints.append(Constraint((x[0] + x[1] + x[32] - 2 * x[33]), label="En"))
    NodeConstraints.append(Constraint((x[2] + x[3] + x[34] - 2 * x[35]), label="En"))
    NodeConstraints.append(Constraint((x[4] + x[5] + x[36] - 2 * x[37]), label="En"))
    NodeConstraints.append(Constraint((x[6] + x[7] + x[38] - 2 * x[39]), label="En"))
    NodeConstraints.append(Constraint((x[8] + x[9] + x[40] - 2 * x[41]), label="En"))
    NodeConstraints.append(Constraint((x[10] + x[11] + x[42] - 2 * x[43]), label="En"))
    NodeConstraints.append(Constraint((x[12] + x[13] + x[44] - 2 * x[45]), label="En"))
    NodeConstraints.append(Constraint((x[14] + x[15] + x[46] - 2 * x[47]), label="En"))
    NodeConstraints.append(Constraint((x[16] + x[17] + x[48] - 2 * x[49]), label="En"))
    NodeConstraints.append(Constraint((x[18] + x[19] + x[50] - 2 * x[51]), label="En"))
    NodeConstraints.append(Constraint((x[20] + x[21] + x[52] - 2 * x[53]), label="En"))
    NodeConstraints.append(Constraint((x[22] + x[23] + x[54] - 2 * x[55]), label="En"))
    NodeConstraints.append(Constraint((x[24] + x[25] + x[56] - 2 * x[57]), label="En"))
    NodeConstraints.append(Constraint((x[26] + x[27] + x[58] - 2 * x[59]), label="En"))
    NodeConstraints.append(Constraint((x[28] + x[29] + x[60] - 2 * x[61]), label="En"))
    NodeConstraints.append(Constraint((x[30] + x[31] + x[62] - 2 * x[63]), label="En"))
    NodeConstraints.append(Constraint((x[32] + x[34] + x[64] - 2 * x[65]), label="En"))
    NodeConstraints.append(Constraint((x[1] + x[3] + x[66] - 2 * x[67]), label="En"))
    NodeConstraints.append(Constraint((x[36] + x[38] + x[68] - 2 * x[69]), label="En"))
    NodeConstraints.append(Constraint((x[5] + x[7] + x[70] - 2 * x[71]), label="En"))
    NodeConstraints.append(Constraint((x[40] + x[42] + x[72] - 2 * x[73]), label="En"))
    NodeConstraints.append(Constraint((x[9] + x[11] + x[74] - 2 * x[75]), label="En"))
    NodeConstraints.append(Constraint((x[44] + x[46] + x[76] - 2 * x[77]), label="En"))
    NodeConstraints.append(Constraint((x[13] + x[15] + x[78] - 2 * x[79]), label="En"))
    NodeConstraints.append(Constraint((x[48] + x[50] + x[80] - 2 * x[81]), label="En"))
    NodeConstraints.append(Constraint((x[17] + x[19] + x[82] - 2 * x[83]), label="En"))
    NodeConstraints.append(Constraint((x[52] + x[54] + x[84] - 2 * x[85]), label="En"))
    NodeConstraints.append(Constraint((x[21] + x[23] + x[86] - 2 * x[87]), label="En"))
    NodeConstraints.append(Constraint((x[56] + x[58] + x[88] - 2 * x[89]), label="En"))
    NodeConstraints.append(Constraint((x[25] + x[27] + x[90] - 2 * x[91]), label="En"))
    NodeConstraints.append(Constraint((x[60] + x[62] + x[92] - 2 * x[93]), label="En"))
    NodeConstraints.append(Constraint((x[29] + x[31] + x[94] - 2 * x[95]), label="En"))
    NodeConstraints.append(Constraint((x[64] + x[68] + x[96] - 2 * x[97]), label="En"))
    NodeConstraints.append(Constraint((x[66] + x[70] + x[98] - 2 * x[99]), label="En"))
    NodeConstraints.append(Constraint((x[34] + x[38] + x[100] - 2 * x[101]), label="En"))
    NodeConstraints.append(Constraint((x[3] + x[7] + x[102] - 2 * x[103]), label="En"))
    NodeConstraints.append(Constraint((x[72] + x[76] + x[104] - 2 * x[105]), label="En"))
    NodeConstraints.append(Constraint((x[74] + x[78] + x[106] - 2 * x[107]), label="En"))
    NodeConstraints.append(Constraint((x[42] + x[46] + x[108] - 2 * x[109]), label="En"))
    NodeConstraints.append(Constraint((x[11] + x[15] + x[110] - 2 * x[111]), label="En"))
    NodeConstraints.append(Constraint((x[80] + x[84] + x[112] - 2 * x[113]), label="En"))
    NodeConstraints.append(Constraint((x[82] + x[86] + x[114] - 2 * x[115]), label="En"))
    NodeConstraints.append(Constraint((x[50] + x[54] + x[116] - 2 * x[117]), label="En"))
    NodeConstraints.append(Constraint((x[19] + x[23] + x[118] - 2 * x[119]), label="En"))
    NodeConstraints.append(Constraint((x[88] + x[92] + x[120] - 2 * x[121]), label="En"))
    NodeConstraints.append(Constraint((x[90] + x[94] + x[122] - 2 * x[123]), label="En"))
    NodeConstraints.append(Constraint((x[58] + x[62] + x[124] - 2 * x[125]), label="En"))
    NodeConstraints.append(Constraint((x[27] + x[31] + x[126] - 2 * x[127]), label="En"))
    NodeConstraints.append(Constraint((x[96] + x[104] + x[128] - 2 * x[129]), label="En"))
    NodeConstraints.append(Constraint((x[98] + x[106] + x[130] - 2 * x[131]), label="En"))
    NodeConstraints.append(Constraint((x[100] + x[108] + x[132] - 2 * x[133]), label="En"))
    NodeConstraints.append(Constraint((x[102] + x[110] + x[134] - 2 * x[135]), label="En"))
    NodeConstraints.append(Constraint((x[68] + x[76] + x[136] - 2 * x[137]), label="En"))
    NodeConstraints.append(Constraint((x[70] + x[78] + x[138] - 2 * x[139]), label="En"))
    NodeConstraints.append(Constraint((x[38] + x[46] + x[140] - 2 * x[141]), label="En"))
    NodeConstraints.append(Constraint((x[7] + x[15] + x[142] - 2 * x[143]), label="En"))
    NodeConstraints.append(Constraint((x[112] + x[120] + x[144] - 2 * x[145]), label="En"))
    NodeConstraints.append(Constraint((x[114] + x[122] + x[146] - 2 * x[147]), label="En"))
    NodeConstraints.append(Constraint((x[116] + x[124] + x[148] - 2 * x[149]), label="En"))
    NodeConstraints.append(Constraint((x[118] + x[126] + x[150] - 2 * x[151]), label="En"))
    NodeConstraints.append(Constraint((x[84] + x[92] + x[152] - 2 * x[153]), label="En"))
    NodeConstraints.append(Constraint((x[86] + x[94] + x[154] - 2 * x[155]), label="En"))
    NodeConstraints.append(Constraint((x[54] + x[62] + x[156] - 2 * x[157]), label="En"))
    NodeConstraints.append(Constraint((x[23] + x[31] + x[158] - 2 * x[159]), label="En"))
    NodeConstraints.append(Constraint((x[128] + x[144] + x[160] - 2 * x[161]), label="En"))
    NodeConstraints.append(Constraint((x[130] + x[146] + x[162] - 2 * x[163]), label="En"))
    NodeConstraints.append(Constraint((x[132] + x[148] + x[164] - 2 * x[165]), label="En"))
    NodeConstraints.append(Constraint((x[134] + x[150] + x[166] - 2 * x[167]), label="En"))
    NodeConstraints.append(Constraint((x[136] + x[152] + x[168] - 2 * x[169]), label="En"))
    NodeConstraints.append(Constraint((x[138] + x[154] + x[170] - 2 * x[171]), label="En"))
    NodeConstraints.append(Constraint((x[140] + x[156] + x[172] - 2 * x[173]), label="En"))
    NodeConstraints.append(Constraint((x[142] + x[158] + x[174] - 2 * x[175]), label="En"))
    NodeConstraints.append(Constraint((x[104] + x[120] + x[176] - 2 * x[177]), label="En"))
    NodeConstraints.append(Constraint((x[106] + x[122] + x[178] - 2 * x[179]), label="En"))
    NodeConstraints.append(Constraint((x[108] + x[124] + x[180] - 2 * x[181]), label="En"))
    NodeConstraints.append(Constraint((x[110] + x[126] + x[182] - 2 * x[183]), label="En"))
    NodeConstraints.append(Constraint((x[76] + x[92] + x[184] - 2 * x[185]), label="En"))
    NodeConstraints.append(Constraint((x[78] + x[94] + x[186] - 2 * x[187]), label="En"))
    NodeConstraints.append(Constraint((x[46] + x[62] + x[188] - 2 * x[189]), label="En"))
    NodeConstraints.append(Constraint((x[15] + x[31] + x[190] - 2 * x[191]), label="En"))
    NodeConstraints = [c * c for c in NodeConstraints]
    constraint_func = We * sum(NodeConstraints)
    cost_func = y[0] * (1 - x[160]) + y[1] * x[160] + y[2] * (1 - x[162]) + y[3] * x[162] + y[4] * (1 - x[164]) + y[5] * \
                x[164] \
                + y[6] * (1 - x[166]) + y[7] * x[166] + y[8] * (1 - x[168]) + y[9] * x[168] + y[10] * (1 - x[170]) + y[
                    11] * x[170] \
                + y[12] * (1 - x[172]) + y[13] * x[172] + y[14] * (1 - x[174]) + y[15] * x[174] + y[16] * (1 - x[176]) + \
                y[17] * x[176] \
                + y[18] * (1 - x[178]) + y[19] * x[178] + y[20] * (1 - x[180]) + y[21] * x[180] + y[22] * (1 - x[182]) + \
                y[23] * x[182] \
                + y[24] * (1 - x[184]) + y[25] * x[184] + y[26] * (1 - x[186]) + y[27] * x[186] + y[28] * (1 - x[188]) + \
                y[29] * x[188] \
                + y[30] * (1 - x[190]) + y[31] * x[190] + y[32] * (1 - x[144]) + y[33] * x[144] + y[34] * (1 - x[146]) + \
                y[35] * x[146] \
                + y[36] * (1 - x[148]) + y[37] * x[148] + y[38] * (1 - x[150]) + y[39] * x[150] + y[40] * (1 - x[152]) + \
                y[41] * x[152] \
                + y[42] * (1 - x[154]) + y[43] * x[154] + y[44] * (1 - x[156]) + y[45] * x[156] + y[46] * (1 - x[158]) + \
                y[47] * x[158] \
                + y[48] * (1 - x[120]) + y[49] * x[120] + y[50] * (1 - x[122]) + y[51] * x[122] + y[52] * (1 - x[124]) + \
                y[53] * x[124] \
                + y[54] * (1 - x[126]) + y[55] * x[126] + y[56] * (1 - x[92]) + y[57] * x[92] + y[58] * (1 - x[94]) + y[
                    59] * x[94] \
                + y[60] * (1 - x[62]) + y[61] * x[62] + y[62] * (1 - x[31]) + y[63] * x[31] + 100 * (
                            x[0] + x[1] + x[2] + x[3] + x[4] + x[5] + x[6] + x[8] + x[9] + x[10] + x[12] + x[16] + x[
                        17] + x[18] + x[20] + x[24])
    H = cost_func + constraint_func
    model = H.compile()
    bqm = model.to_bqm()
    sa = neal.SimulatedAnnealingSampler()
    sampleset = sa.sample(bqm, num_reads=300)
    decoded_samples = model.decode_sampleset(sampleset)
    best_sample = min(decoded_samples, key=lambda x: x.energy)
    result = [0, 0]
    for i in range(K):
        if line[i] != str(best_sample.sample[a[index[i]]]):
            result[0] = result[0] + 1
    raw_solution = best_sample.sample
    decoded_sample = model.decode_sample(raw_solution, vartype='BINARY')
    for c in decoded_sample.constraints(only_broken=True):
        if c[0] == 'E':
            result[1] = 1
    q.put(result)


def thr14(y, line, N, K, n_var, a, We, q, index):
    x = [Binary(a[i]) for i in range(n_var)]
    NodeConstraints = []
    NodeConstraints.append(Constraint((x[0] + x[1] + x[32] - 2 * x[33]), label="En"))
    NodeConstraints.append(Constraint((x[2] + x[3] + x[34] - 2 * x[35]), label="En"))
    NodeConstraints.append(Constraint((x[4] + x[5] + x[36] - 2 * x[37]), label="En"))
    NodeConstraints.append(Constraint((x[6] + x[7] + x[38] - 2 * x[39]), label="En"))
    NodeConstraints.append(Constraint((x[8] + x[9] + x[40] - 2 * x[41]), label="En"))
    NodeConstraints.append(Constraint((x[10] + x[11] + x[42] - 2 * x[43]), label="En"))
    NodeConstraints.append(Constraint((x[12] + x[13] + x[44] - 2 * x[45]), label="En"))
    NodeConstraints.append(Constraint((x[14] + x[15] + x[46] - 2 * x[47]), label="En"))
    NodeConstraints.append(Constraint((x[16] + x[17] + x[48] - 2 * x[49]), label="En"))
    NodeConstraints.append(Constraint((x[18] + x[19] + x[50] - 2 * x[51]), label="En"))
    NodeConstraints.append(Constraint((x[20] + x[21] + x[52] - 2 * x[53]), label="En"))
    NodeConstraints.append(Constraint((x[22] + x[23] + x[54] - 2 * x[55]), label="En"))
    NodeConstraints.append(Constraint((x[24] + x[25] + x[56] - 2 * x[57]), label="En"))
    NodeConstraints.append(Constraint((x[26] + x[27] + x[58] - 2 * x[59]), label="En"))
    NodeConstraints.append(Constraint((x[28] + x[29] + x[60] - 2 * x[61]), label="En"))
    NodeConstraints.append(Constraint((x[30] + x[31] + x[62] - 2 * x[63]), label="En"))
    NodeConstraints.append(Constraint((x[32] + x[34] + x[64] - 2 * x[65]), label="En"))
    NodeConstraints.append(Constraint((x[1] + x[3] + x[66] - 2 * x[67]), label="En"))
    NodeConstraints.append(Constraint((x[36] + x[38] + x[68] - 2 * x[69]), label="En"))
    NodeConstraints.append(Constraint((x[5] + x[7] + x[70] - 2 * x[71]), label="En"))
    NodeConstraints.append(Constraint((x[40] + x[42] + x[72] - 2 * x[73]), label="En"))
    NodeConstraints.append(Constraint((x[9] + x[11] + x[74] - 2 * x[75]), label="En"))
    NodeConstraints.append(Constraint((x[44] + x[46] + x[76] - 2 * x[77]), label="En"))
    NodeConstraints.append(Constraint((x[13] + x[15] + x[78] - 2 * x[79]), label="En"))
    NodeConstraints.append(Constraint((x[48] + x[50] + x[80] - 2 * x[81]), label="En"))
    NodeConstraints.append(Constraint((x[17] + x[19] + x[82] - 2 * x[83]), label="En"))
    NodeConstraints.append(Constraint((x[52] + x[54] + x[84] - 2 * x[85]), label="En"))
    NodeConstraints.append(Constraint((x[21] + x[23] + x[86] - 2 * x[87]), label="En"))
    NodeConstraints.append(Constraint((x[56] + x[58] + x[88] - 2 * x[89]), label="En"))
    NodeConstraints.append(Constraint((x[25] + x[27] + x[90] - 2 * x[91]), label="En"))
    NodeConstraints.append(Constraint((x[60] + x[62] + x[92] - 2 * x[93]), label="En"))
    NodeConstraints.append(Constraint((x[29] + x[31] + x[94] - 2 * x[95]), label="En"))
    NodeConstraints.append(Constraint((x[64] + x[68] + x[96] - 2 * x[97]), label="En"))
    NodeConstraints.append(Constraint((x[66] + x[70] + x[98] - 2 * x[99]), label="En"))
    NodeConstraints.append(Constraint((x[34] + x[38] + x[100] - 2 * x[101]), label="En"))
    NodeConstraints.append(Constraint((x[3] + x[7] + x[102] - 2 * x[103]), label="En"))
    NodeConstraints.append(Constraint((x[72] + x[76] + x[104] - 2 * x[105]), label="En"))
    NodeConstraints.append(Constraint((x[74] + x[78] + x[106] - 2 * x[107]), label="En"))
    NodeConstraints.append(Constraint((x[42] + x[46] + x[108] - 2 * x[109]), label="En"))
    NodeConstraints.append(Constraint((x[11] + x[15] + x[110] - 2 * x[111]), label="En"))
    NodeConstraints.append(Constraint((x[80] + x[84] + x[112] - 2 * x[113]), label="En"))
    NodeConstraints.append(Constraint((x[82] + x[86] + x[114] - 2 * x[115]), label="En"))
    NodeConstraints.append(Constraint((x[50] + x[54] + x[116] - 2 * x[117]), label="En"))
    NodeConstraints.append(Constraint((x[19] + x[23] + x[118] - 2 * x[119]), label="En"))
    NodeConstraints.append(Constraint((x[88] + x[92] + x[120] - 2 * x[121]), label="En"))
    NodeConstraints.append(Constraint((x[90] + x[94] + x[122] - 2 * x[123]), label="En"))
    NodeConstraints.append(Constraint((x[58] + x[62] + x[124] - 2 * x[125]), label="En"))
    NodeConstraints.append(Constraint((x[27] + x[31] + x[126] - 2 * x[127]), label="En"))
    NodeConstraints.append(Constraint((x[96] + x[104] + x[128] - 2 * x[129]), label="En"))
    NodeConstraints.append(Constraint((x[98] + x[106] + x[130] - 2 * x[131]), label="En"))
    NodeConstraints.append(Constraint((x[100] + x[108] + x[132] - 2 * x[133]), label="En"))
    NodeConstraints.append(Constraint((x[102] + x[110] + x[134] - 2 * x[135]), label="En"))
    NodeConstraints.append(Constraint((x[68] + x[76] + x[136] - 2 * x[137]), label="En"))
    NodeConstraints.append(Constraint((x[70] + x[78] + x[138] - 2 * x[139]), label="En"))
    NodeConstraints.append(Constraint((x[38] + x[46] + x[140] - 2 * x[141]), label="En"))
    NodeConstraints.append(Constraint((x[7] + x[15] + x[142] - 2 * x[143]), label="En"))
    NodeConstraints.append(Constraint((x[112] + x[120] + x[144] - 2 * x[145]), label="En"))
    NodeConstraints.append(Constraint((x[114] + x[122] + x[146] - 2 * x[147]), label="En"))
    NodeConstraints.append(Constraint((x[116] + x[124] + x[148] - 2 * x[149]), label="En"))
    NodeConstraints.append(Constraint((x[118] + x[126] + x[150] - 2 * x[151]), label="En"))
    NodeConstraints.append(Constraint((x[84] + x[92] + x[152] - 2 * x[153]), label="En"))
    NodeConstraints.append(Constraint((x[86] + x[94] + x[154] - 2 * x[155]), label="En"))
    NodeConstraints.append(Constraint((x[54] + x[62] + x[156] - 2 * x[157]), label="En"))
    NodeConstraints.append(Constraint((x[23] + x[31] + x[158] - 2 * x[159]), label="En"))
    NodeConstraints.append(Constraint((x[128] + x[144] + x[160] - 2 * x[161]), label="En"))
    NodeConstraints.append(Constraint((x[130] + x[146] + x[162] - 2 * x[163]), label="En"))
    NodeConstraints.append(Constraint((x[132] + x[148] + x[164] - 2 * x[165]), label="En"))
    NodeConstraints.append(Constraint((x[134] + x[150] + x[166] - 2 * x[167]), label="En"))
    NodeConstraints.append(Constraint((x[136] + x[152] + x[168] - 2 * x[169]), label="En"))
    NodeConstraints.append(Constraint((x[138] + x[154] + x[170] - 2 * x[171]), label="En"))
    NodeConstraints.append(Constraint((x[140] + x[156] + x[172] - 2 * x[173]), label="En"))
    NodeConstraints.append(Constraint((x[142] + x[158] + x[174] - 2 * x[175]), label="En"))
    NodeConstraints.append(Constraint((x[104] + x[120] + x[176] - 2 * x[177]), label="En"))
    NodeConstraints.append(Constraint((x[106] + x[122] + x[178] - 2 * x[179]), label="En"))
    NodeConstraints.append(Constraint((x[108] + x[124] + x[180] - 2 * x[181]), label="En"))
    NodeConstraints.append(Constraint((x[110] + x[126] + x[182] - 2 * x[183]), label="En"))
    NodeConstraints.append(Constraint((x[76] + x[92] + x[184] - 2 * x[185]), label="En"))
    NodeConstraints.append(Constraint((x[78] + x[94] + x[186] - 2 * x[187]), label="En"))
    NodeConstraints.append(Constraint((x[46] + x[62] + x[188] - 2 * x[189]), label="En"))
    NodeConstraints.append(Constraint((x[15] + x[31] + x[190] - 2 * x[191]), label="En"))
    NodeConstraints = [c * c for c in NodeConstraints]
    constraint_func = We * sum(NodeConstraints)
    cost_func = y[0] * (1 - x[160]) + y[1] * x[160] + y[2] * (1 - x[162]) + y[3] * x[162] + y[4] * (1 - x[164]) + y[5] * \
                x[164] \
                + y[6] * (1 - x[166]) + y[7] * x[166] + y[8] * (1 - x[168]) + y[9] * x[168] + y[10] * (1 - x[170]) + y[
                    11] * x[170] \
                + y[12] * (1 - x[172]) + y[13] * x[172] + y[14] * (1 - x[174]) + y[15] * x[174] + y[16] * (1 - x[176]) + \
                y[17] * x[176] \
                + y[18] * (1 - x[178]) + y[19] * x[178] + y[20] * (1 - x[180]) + y[21] * x[180] + y[22] * (1 - x[182]) + \
                y[23] * x[182] \
                + y[24] * (1 - x[184]) + y[25] * x[184] + y[26] * (1 - x[186]) + y[27] * x[186] + y[28] * (1 - x[188]) + \
                y[29] * x[188] \
                + y[30] * (1 - x[190]) + y[31] * x[190] + y[32] * (1 - x[144]) + y[33] * x[144] + y[34] * (1 - x[146]) + \
                y[35] * x[146] \
                + y[36] * (1 - x[148]) + y[37] * x[148] + y[38] * (1 - x[150]) + y[39] * x[150] + y[40] * (1 - x[152]) + \
                y[41] * x[152] \
                + y[42] * (1 - x[154]) + y[43] * x[154] + y[44] * (1 - x[156]) + y[45] * x[156] + y[46] * (1 - x[158]) + \
                y[47] * x[158] \
                + y[48] * (1 - x[120]) + y[49] * x[120] + y[50] * (1 - x[122]) + y[51] * x[122] + y[52] * (1 - x[124]) + \
                y[53] * x[124] \
                + y[54] * (1 - x[126]) + y[55] * x[126] + y[56] * (1 - x[92]) + y[57] * x[92] + y[58] * (1 - x[94]) + y[
                    59] * x[94] \
                + y[60] * (1 - x[62]) + y[61] * x[62] + y[62] * (1 - x[31]) + y[63] * x[31] + 100 * (
                            x[0] + x[1] + x[2] + x[3] + x[4] + x[5] + x[6] + x[8] + x[9] + x[10] + x[12] + x[16] + x[
                        17] + x[18] + x[20] + x[24])
    H = cost_func + constraint_func
    model = H.compile()
    bqm = model.to_bqm()
    sa = neal.SimulatedAnnealingSampler()
    sampleset = sa.sample(bqm, num_reads=300)
    decoded_samples = model.decode_sampleset(sampleset)
    best_sample = min(decoded_samples, key=lambda x: x.energy)
    result = [0, 0]
    for i in range(K):
        if line[i] != str(best_sample.sample[a[index[i]]]):
            result[0] = result[0] + 1
    raw_solution = best_sample.sample
    decoded_sample = model.decode_sample(raw_solution, vartype='BINARY')
    for c in decoded_sample.constraints(only_broken=True):
        if c[0] == 'E':
            result[1] = 1
    q.put(result)


def thr15(y, line, N, K, n_var, a, We, q, index):
    x = [Binary(a[i]) for i in range(n_var)]
    NodeConstraints = []
    NodeConstraints.append(Constraint((x[0] + x[1] + x[32] - 2 * x[33]), label="En"))
    NodeConstraints.append(Constraint((x[2] + x[3] + x[34] - 2 * x[35]), label="En"))
    NodeConstraints.append(Constraint((x[4] + x[5] + x[36] - 2 * x[37]), label="En"))
    NodeConstraints.append(Constraint((x[6] + x[7] + x[38] - 2 * x[39]), label="En"))
    NodeConstraints.append(Constraint((x[8] + x[9] + x[40] - 2 *
 x[41]), label="En"))
    NodeConstraints.append(Constraint((x[10] + x[11] + x[42] - 2 * x[43]), label="En"))
    NodeConstraints.append(Constraint((x[12] + x[13] + x[44] - 2 * x[45]), label="En"))
    NodeConstraints.append(Constraint((x[14] + x[15] + x[46] - 2 * x[47]), label="En"))
    NodeConstraints.append(Constraint((x[16] + x[17] + x[48] - 2 * x[49]), label="En"))
    NodeConstraints.append(Constraint((x[18] + x[19] + x[50] - 2 * x[51]), label="En"))
    NodeConstraints.append(Constraint((x[20] + x[21] + x[52] - 2 * x[53]), label="En"))
    NodeConstraints.append(Constraint((x[22] + x[23] + x[54] - 2 * x[55]), label="En"))
    NodeConstraints.append(Constraint((x[24] + x[25] + x[56] - 2 * x[57]), label="En"))
    NodeConstraints.append(Constraint((x[26] + x[27] + x[58] - 2 * x[59]), label="En"))
    NodeConstraints.append(Constraint((x[28] + x[29] + x[60] - 2 * x[61]), label="En"))
    NodeConstraints.append(Constraint((x[30] + x[31] + x[62] - 2 * x[63]), label="En"))
    NodeConstraints.append(Constraint((x[32] + x[34] + x[64] - 2 * x[65]), label="En"))
    NodeConstraints.append(Constraint((x[1] + x[3] + x[66] - 2 * x[67]), label="En"))
    NodeConstraints.append(Constraint((x[36] + x[38] + x[68] - 2 * x[69]), label="En"))
    NodeConstraints.append(Constraint((x[5] + x[7] + x[70] - 2 * x[71]), label="En"))
    NodeConstraints.append(Constraint((x[40] + x[42] + x[72] - 2 * x[73]), label="En"))
    NodeConstraints.append(Constraint((x[9] + x[11] + x[74] - 2 * x[75]), label="En"))
    NodeConstraints.append(Constraint((x[44] + x[46] + x[76] - 2 * x[77]), label="En"))
    NodeConstraints.append(Constraint((x[13] + x[15] + x[78] - 2 * x[79]), label="En"))
    NodeConstraints.append(Constraint((x[48] + x[50] + x[80] - 2 * x[81]), label="En"))
    NodeConstraints.append(Constraint((x[17] + x[19] + x[82] - 2 * x[83]), label="En"))
    NodeConstraints.append(Constraint((x[52] + x[54] + x[84] - 2 * x[85]), label="En"))
    NodeConstraints.append(Constraint((x[21] + x[23] + x[86] - 2 * x[87]), label="En"))
    NodeConstraints.append(Constraint((x[56] + x[58] + x[88] - 2 * x[89]), label="En"))
    NodeConstraints.append(Constraint((x[25] + x[27] + x[90] - 2 * x[91]), label="En"))
    NodeConstraints.append(Constraint((x[60] + x[62] + x[92] - 2 * x[93]), label="En"))
    NodeConstraints.append(Constraint((x[29] + x[31] + x[94] - 2 * x[95]), label="En"))
    NodeConstraints.append(Constraint((x[64] + x[68] + x[96] - 2 * x[97]), label="En"))
    NodeConstraints.append(Constraint((x[66] + x[70] + x[98] - 2 * x[99]), label="En"))
    NodeConstraints.append(Constraint((x[34] + x[38] + x[100] - 2 * x[101]), label="En"))
    NodeConstraints.append(Constraint((x[3] + x[7] + x[102] - 2 * x[103]), label="En"))
    NodeConstraints.append(Constraint((x[72] + x[76] + x[104] - 2 * x[105]), label="En"))
    NodeConstraints.append(Constraint((x[74] + x[78] + x[106] - 2 * x[107]), label="En"))
    NodeConstraints.append(Constraint((x[42] + x[46] + x[108] - 2 * x[109]), label="En"))
    NodeConstraints.append(Constraint((x[11] + x[15] + x[110] - 2 * x[111]), label="En"))
    NodeConstraints.append(Constraint((x[80] + x[84] + x[112] - 2 * x[113]), label="En"))
    NodeConstraints.append(Constraint((x[82] + x[86] + x[114] - 2 * x[115]), label="En"))
    NodeConstraints.append(Constraint((x[50] + x[54] + x[116] - 2 * x[117]), label="En"))
    NodeConstraints.append(Constraint((x[19] + x[23] + x[118] - 2 * x[119]), label="En"))
    NodeConstraints.append(Constraint((x[88] + x[92] + x[120] - 2 * x[121]), label="En"))
    NodeConstraints.append(Constraint((x[90] + x[94] + x[122] - 2 * x[123]), label="En"))
    NodeConstraints.append(Constraint((x[58] + x[62] + x[124] - 2 * x[125]), label="En"))
    NodeConstraints.append(Constraint((x[27] + x[31] + x[126] - 2 * x[127]), label="En"))
    NodeConstraints.append(Constraint((x[96] + x[104] + x[128] - 2 * x[129]), label="En"))
    NodeConstraints.append(Constraint((x[98] + x[106] + x[130] - 2 * x[131]), label="En"))
    NodeConstraints.append(Constraint((x[100] + x[108] + x[132] - 2 * x[133]), label="En"))
    NodeConstraints.append(Constraint((x[102] + x[110] + x[134] - 2 * x[135]), label="En"))
    NodeConstraints.append(Constraint((x[68] + x[76] + x[136] - 2 * x[137]), label="En"))
    NodeConstraints.append(Constraint((x[70] + x[78] + x[138] - 2 * x[139]), label="En"))
    NodeConstraints.append(Constraint((x[38] + x[46] + x[140] - 2 * x[141]), label="En"))
    NodeConstraints.append(Constraint((x[7] + x[15] + x[142] - 2 * x[143]), label="En"))
    NodeConstraints.append(Constraint((x[112] + x[120] + x[144] - 2 * x[145]), label="En"))
    NodeConstraints.append(Constraint((x[114] + x[122] + x[146] - 2 * x[147]), label="En"))
    NodeConstraints.append(Constraint((x[116] + x[124] + x[148] - 2 * x[149]), label="En"))
    NodeConstraints.append(Constraint((x[118] + x[126] + x[150] - 2 * x[151]), label="En"))
    NodeConstraints.append(Constraint((x[84] + x[92] + x[152] - 2 * x[153]), label="En"))
    NodeConstraints.append(Constraint((x[86] + x[94] + x[154] - 2 * x[155]), label="En"))
    NodeConstraints.append(Constraint((x[54] + x[62] + x[156] - 2 * x[157]), label="En"))
    NodeConstraints.append(Constraint((x[23] + x[31] + x[158] - 2 * x[159]), label="En"))
    NodeConstraints.append(Constraint((x[128] + x[144] + x[160] - 2 * x[161]), label="En"))
    NodeConstraints.append(Constraint((x[130] + x[146] + x[162] - 2 * x[163]), label="En"))
    NodeConstraints.append(Constraint((x[132] + x[148] + x[164] - 2 * x[165]), label="En"))
    NodeConstraints.append(Constraint((x[134] + x[150] + x[166] - 2 * x[167]), label="En"))
    NodeConstraints.append(Constraint((x[136] + x[152] + x[168] - 2 * x[169]), label="En"))
    NodeConstraints.append(Constraint((x[138] + x[154] + x[170] - 2 * x[171]), label="En"))
    NodeConstraints.append(Constraint((x[140] + x[156] + x[172] - 2 * x[173]), label="En"))
    NodeConstraints.append(Constraint((x[142] + x[158] + x[174] - 2 * x[175]), label="En"))
    NodeConstraints.append(Constraint((x[104] + x[120] + x[176] - 2 * x[177]), label="En"))
    NodeConstraints.append(Constraint((x[106] + x[122] + x[178] - 2 * x[179]), label="En"))
    NodeConstraints.append(Constraint((x[108] + x[124] + x[180] - 2 * x[181]), label="En"))
    NodeConstraints.append(Constraint((x[110] + x[126] + x[182] - 2 * x[183]), label="En"))
    NodeConstraints.append(Constraint((x[76] + x[92] + x[184] - 2 * x[185]), label="En"))
    NodeConstraints.append(Constraint((x[78] + x[94] + x[186] - 2 * x[187]), label="En"))
    NodeConstraints.append(Constraint((x[46] + x[62] + x[188] - 2 * x[189]), label="En"))
    NodeConstraints.append(Constraint((x[15] + x[31] + x[190] - 2 * x[191]), label="En"))
    NodeConstraints = [c * c for c in NodeConstraints]
    constraint_func = We * sum(NodeConstraints)
    cost_func = y[0] * (1 - x[160]) + y[1] * x[160] + y[2] * (1 - x[162]) + y[3] * x[162] + y[4] * (1 - x[164]) + y[5] * \
                x[164] \
                + y[6] * (1 - x[166]) + y[7] * x[166] + y[8] * (1 - x[168]) + y[9] * x[168] + y[10] * (1 - x[170]) + y[
                    11] * x[170] \
                + y[12] * (1 - x[172]) + y[13] * x[172] + y[14] * (1 - x[174]) + y[15] * x[174] + y[16] * (1 - x[176]) + \
                y[17] * x[176] \
                + y[18] * (1 - x[178]) + y[19] * x[178] + y[20] * (1 - x[180]) + y[21] * x[180] + y[22] * (1 - x[182]) + \
                y[23] * x[182] \
                + y[24] * (1 - x[184]) + y[25] * x[184] + y[26] * (1 - x[186]) + y[27] * x[186] + y[28] * (1 - x[188]) + \
                y[29] * x[188] \
                + y[30] * (1 - x[190]) + y[31] * x[190] + y[32] * (1 - x[144]) + y[33] * x[144] + y[34] * (1 - x[146]) + \
                y[35] * x[146] \
                + y[36] * (1 - x[148]) + y[37] * x[148] + y[38] * (1 - x[150]) + y[39] * x[150] + y[40] * (1 - x[152]) + \
                y[41] * x[152] \
                + y[42] * (1 - x[154]) + y[43] * x[154] + y[44] * (1 - x[156]) + y[45] * x[156] + y[46] * (1 - x[158]) + \
                y[47] * x[158] \
                + y[48] * (1 - x[120]) + y[49] * x[120] + y[50] * (1 - x[122]) + y[51] * x[122] + y[52] * (1 - x[124]) + \
                y[53] * x[124] \
                + y[54] * (1 - x[126]) + y[55] * x[126] + y[56] * (1 - x[92]) + y[57] * x[92] + y[58] * (1 - x[94]) + y[
                    59] * x[94] \
                + y[60] * (1 - x[62]) + y[61] * x[62] + y[62] * (1 - x[31]) + y[63] * x[31] + 100 * (
                            x[0] + x[1] + x[2] + x[3] + x[4] + x[5] + x[6] + x[8] + x[9] + x[10] + x[12] + x[16] + x[
                        17] + x[18] + x[20] + x[24])
    H = cost_func + constraint_func
    model = H.compile()
    bqm = model.to_bqm()
    sa = neal.SimulatedAnnealingSampler()
    sampleset = sa.sample(bqm, num_reads=300)
    decoded_samples = model.decode_sampleset(sampleset)
    best_sample = min(decoded_samples, key=lambda x: x.energy)
    result = [0, 0]
    for i in range(K):
        if line[i] != str(best_sample.sample[a[index[i]]]):
            result[0] = result[0] + 1
    raw_solution = best_sample.sample
    decoded_sample = model.decode_sample(raw_solution, vartype='BINARY')
    for c in decoded_sample.constraints(only_broken=True):
        if c[0] == 'E':
            result[1] = 1
    q.put(result)


def thr16(y, line, N, K, n_var, a, We, q, index):
    x = [Binary(a[i]) for i in range(n_var)]
    NodeConstraints = []
    NodeConstraints.append(Constraint((x[0] + x[1] + x[32] - 2 * x[33]), label="En"))
    NodeConstraints.append(Constraint((x[2] + x[3] + x[34] - 2 * x[35]), label="En"))
    NodeConstraints.append(Constraint((x[4] + x[5] + x[36] - 2 * x[37]), label="En"))
    NodeConstraints.append(Constraint((x[6] + x[7] + x[38] - 2 * x[39]), label="En"))
    NodeConstraints.append(Constraint((x[8] + x[9] + x[40] - 2 * x[41]), label="En"))
    NodeConstraints.append(Constraint((x[10] + x[11] + x[42] - 2 * x[43]), label="En"))
    NodeConstraints.append(Constraint((x[12] + x[13] + x[44] - 2 * x[45]), label="En"))
    NodeConstraints.append(Constraint((x[14] + x[15] + x[46] - 2 * x[47]), label="En"))
    NodeConstraints.append(Constraint((x[16] + x[17] + x[48] - 2 * x[49]), label="En"))
    NodeConstraints.append(Constraint((x[18] + x[19] + x[50] - 2 * x[51]), label="En"))
    NodeConstraints.append(Constraint((x[20] + x[21] + x[52] - 2 * x[53]), label="En"))
    NodeConstraints.append(Constraint((x[22] + x[23] + x[54] - 2 * x[55]), label="En"))
    NodeConstraints.append(Constraint((x[24] + x[25] + x[56] - 2 * x[57]), label="En"))
    NodeConstraints.append(Constraint((x[26] + x[27] + x[58] - 2 * x[59]), label="En"))
    NodeConstraints.append(Constraint((x[28] + x[29] + x[60] - 2 * x[61]), label="En"))
    NodeConstraints.append(Constraint((x[30] + x[31] + x[62] - 2 * x[63]), label="En"))
    NodeConstraints.append(Constraint((x[32] + x[34] + x[64] - 2 * x[65]), label="En"))
    NodeConstraints.append(Constraint((x[1] + x[3] + x[66] - 2 * x[67]), label="En"))
    NodeConstraints.append(Constraint((x[36] + x[38] + x[68] - 2 * x[69]), label="En"))
    NodeConstraints.append(Constraint((x[5] + x[7] + x[70] - 2 * x[71]), label="En"))
    NodeConstraints.append(Constraint((x[40] + x[42] + x[72] - 2 * x[73]), label="En"))
    NodeConstraints.append(Constraint((x[9] + x[11] + x[74] - 2 * x[75]), label="En"))
    NodeConstraints.append(Constraint((x[44] + x[46] + x[76] - 2 * x[77]), label="En"))
    NodeConstraints.append(Constraint((x[13] + x[15] + x[78] - 2 * x[79]), label="En"))
    NodeConstraints.append(Constraint((x[48] + x[50] + x[80] - 2 * x[81]), label="En"))
    NodeConstraints.append(Constraint((x[17] + x[19] + x[82] - 2 * x[83]), label="En"))
    NodeConstraints.append(Constraint((x[52] + x[54] + x[84] - 2 * x[85]), label="En"))
    NodeConstraints.append(Constraint((x[21] + x[23] + x[86] - 2 * x[87]), label="En"))
    NodeConstraints.append(Constraint((x[56] + x[58] + x[88] - 2 * x[89]), label="En"))
    NodeConstraints.append(Constraint((x[25] + x[27] + x[90] - 2 * x[91]), label="En"))
    NodeConstraints.append(Constraint((x[60] + x[62] + x[92] - 2 * x[93]), label="En"))
    NodeConstraints.append(Constraint((x[29] + x[31] + x[94] - 2 * x[95]), label="En"))
    NodeConstraints.append(Constraint((x[64] + x[68] + x[96] - 2 * x[97]), label="En"))
    NodeConstraints.append(Constraint((x[66] + x[70] + x[98] - 2 * x[99]), label="En"))
    NodeConstraints.append(Constraint((x[34] + x[38] + x[100] - 2 * x[101]), label="En"))
    NodeConstraints.append(Constraint((x[3] + x[7] + x[102] - 2 * x[103]), label="En"))
    NodeConstraints.append(Constraint((x[72] + x[76] + x[104] - 2 * x[105]), label="En"))
    NodeConstraints.append(Constraint((x[74] + x[78] + x[106] - 2 * x[107]), label="En"))
    NodeConstraints.append(Constraint((x[42] + x[46] + x[108] - 2 * x[109]), label="En"))
    NodeConstraints.append(Constraint((x[11] + x[15] + x[110] - 2 * x[111]), label="En"))
    NodeConstraints.append(Constraint((x[80] + x[84] + x[112] - 2 * x[113]), label="En"))
    NodeConstraints.append(Constraint((x[82] + x[86] + x[114] - 2 * x[115]), label="En"))
    NodeConstraints.append(Constraint((x[50] + x[54] + x[116] - 2 * x[117]), label="En"))
    NodeConstraints.append(Constraint((x[19] + x[23] + x[118] - 2 * x[119]), label="En"))
    NodeConstraints.append(Constraint((x[88] + x[92] + x[120] - 2 * x[121]), label="En"))
    NodeConstraints.append(Constraint((x[90] + x[94] + x[122] - 2 * x[123]), label="En"))
    NodeConstraints.append(Constraint((x[58] + x[62] + x[124] - 2 * x[125]), label="En"))
    NodeConstraints.append(Constraint((x[27] + x[31] + x[126] - 2 * x[127]), label="En"))
    NodeConstraints.append(Constraint((x[96] + x[104] + x[128] - 2 * x[129]), label="En"))
    NodeConstraints.append(Constraint((x[98] + x[106] + x[130] - 2 * x[131]), label="En"))
    NodeConstraints.append(Constraint((x[100] + x[108] + x[132] - 2 * x[133]), label="En"))
    NodeConstraints.append(Constraint((x[102] + x[110] + x[134] - 2 * x[135]), label="En"))
    NodeConstraints.append(Constraint((x[68] + x[76] + x[136] - 2 * x[137]), label="En"))
    NodeConstraints.append(Constraint((x[70] + x[78] + x[138] - 2 * x[139]), label="En"))
    NodeConstraints.append(Constraint((x[38] + x[46] + x[140] - 2 * x[141]), label="En"))
    NodeConstraints.append(Constraint((x[7] + x[15] + x[142] - 2 * x[143]), label="En"))
    NodeConstraints.append(Constraint((x[112] + x[120] + x[144] - 2 * x[145]), label="En"))
    NodeConstraints.append(Constraint((x[114] + x[122] + x[146] - 2 * x[147]), label="En"))
    NodeConstraints.append(Constraint((x[116] + x[124] + x[148] - 2 * x[149]), label="En"))
    NodeConstraints.append(Constraint((x[118] + x[126] + x[150] - 2 * x[151]), label="En"))
    NodeConstraints.append(Constraint((x[84] + x[92] + x[152] - 2 * x[153]), label="En"))
    NodeConstraints.append(Constraint((x[86] + x[94] + x[154] - 2 * x[155]), label="En"))
    NodeConstraints.append(Constraint((x[54] + x[62] + x[156] - 2 * x[157]), label="En"))
    NodeConstraints.append(Constraint((x[23] + x[31] + x[158] - 2 * x[159]), label="En"))
    NodeConstraints.append(Constraint((x[128] + x[144] + x[160] - 2 * x[161]), label="En"))
    NodeConstraints.append(Constraint((x[130] + x[146] + x[162] - 2 * x[163]), label="En"))
    NodeConstraints.append(Constraint((x[132] + x[148] + x[164] - 2 * x[165]), label="En"))
    NodeConstraints.append(Constraint((x[134] + x[150] + x[166] - 2 * x[167]), label="En"))
    NodeConstraints.append(Constraint((x[136] + x[152] + x[168] - 2 * x[169]), label="En"))
    NodeConstraints.append(Constraint((x[138] + x[154] + x[170] - 2 * x[171]), label="En"))
    NodeConstraints.append(Constraint((x[140] + x[156] + x[172] - 2 * x[173]), label="En"))
    NodeConstraints.append(Constraint((x[142] + x[158] + x[174] - 2 * x[175]), label="En"))
    NodeConstraints.append(Constraint((x[104] + x[120] + x[176] - 2 * x[177]), label="En"))
    NodeConstraints.append(Constraint((x[106] + x[122] + x[178] - 2 * x[179]), label="En"))
    NodeConstraints.append(Constraint((x[108] + x[124] + x[180] - 2 * x[181]), label="En"))
    NodeConstraints.append(Constraint((x[110] + x[126] + x[182] - 2 * x[183]), label="En"))
    NodeConstraints.append(Constraint((x[76] + x[92] + x[184] - 2 * x[185]), label="En"))
    NodeConstraints.append(Constraint((x[78] + x[94] + x[186] - 2 * x[187]), label="En"))
    NodeConstraints.append(Constraint((x[46] + x[62] + x[188] - 2 * x[189]), label="En"))
    NodeConstraints.append(Constraint((x[15] + x[31] + x[190] - 2 * x[191]), label="En"))
    NodeConstraints = [c * c for c in NodeConstraints]
    constraint_func = We * sum(NodeConstraints)
    cost_func = y[0] * (1 - x[160]) + y[1] * x[160] + y[2] * (1 - x[162]) + y[3] * x[162] + y[4] * (1 - x[164]) + y[5] * \
                x[164] \
                + y[6] * (1 - x[166]) + y[7] * x[166] + y[8] * (1 - x[168]) + y[9] * x[168] + y[10] * (1 - x[170]) + y[
                    11] * x[170] \
                + y[12] * (1 - x[172]) + y[13] * x[172] + y[14] * (1 - x[174]) + y[15] * x[174] + y[16] * (1 - x[176]) + \
                y[17] * x[176] \
                + y[18] * (1 - x[178]) + y[19] * x[178] + y[20] * (1 - x[180]) + y[21] * x[180] + y[22] * (1 - x[182]) + \
                y[23] * x[182] \
                + y[24] * (1 - x[184]) + y[25] * x[184] + y[26] * (1 - x[186]) + y[27] * x[186] + y[28] * (1 - x[188]) + \
                y[29] * x[188] \
                + y[30] * (1 - x[190]) + y[31] * x[190] + y[32] * (1 - x[144]) + y[33] * x[144] + y[34] * (1 - x[146]) + \
                y[35] * x[146] \
                + y[36] * (1 - x[148]) + y[37] * x[148] + y[38] * (1 - x[150]) + y[39] * x[150] + y[40] * (1 - x[152]) + \
                y[41] * x[152] \
                + y[42] * (1 - x[154]) + y[43] * x[154] + y[44] * (1 - x[156]) + y[45] * x[156] + y[46] * (1 - x[158]) + \
                y[47] * x[158] \
                + y[48] * (1 - x[120]) + y[49] * x[120] + y[50] * (1 - x[122]) + y[51] * x[122] + y[52] * (1 - x[124]) + \
                y[53] * x[124] \
                + y[54] * (1 - x[126]) + y[55] * x[126] + y[56] * (1 - x[92]) + y[57] * x[92] + y[58] * (1 - x[94]) + y[
                    59] * x[94] \
                + y[60] * (1 - x[62]) + y[61] * x[62] + y[62] * (1 - x[31]) + y[63] * x[31] + 100 * (
                            x[0] + x[1] + x[2] + x[3] + x[4] + x[5] + x[6] + x[8] + x[9] + x[10] + x[12] + x[16] + x[
                        17] + x[18] + x[20] + x[24])
    H = cost_func + constraint_func
    model = H.compile()
    bqm = model.to_bqm()
    sa = neal.SimulatedAnnealingSampler()
    sampleset = sa.sample(bqm, num_reads=300)
    decoded_samples = model.decode_sampleset(sampleset)
    best_sample = min(decoded_samples, key=lambda x: x.energy)
    result = [0, 0]
    for i in range(K):
        if line[i] != str(best_sample.sample[a[index[i]]]):
            result[0] = result[0] + 1
    raw_solution = best_sample.sample
    decoded_sample = model.decode_sample(raw_solution, vartype='BINARY')
    for c in decoded_sample.constraints(only_broken=True):
        if c[0] == 'E':
            result[1] = 1
    q.put(result)


if __name__ == "__main__":
    f = open("3216-1-5.in")
    N = 32
    K = 16
    n_var = 192
    index = [7, 11, 13, 14, 15, 19, 21, 22, 23, 25, 26, 27, 28, 29, 30, 31]
    a = {}
    for i in range(n_var):
        a[i] = ''
        if i < 1000:
            a[i] = '0'
        if i < 100:
            a[i] = '00'
        if i < 10:
            a[i] = '000'
        a[i] = a[i] + str(i)
    We = 5
    print(We)
    Numsnr = int(f.readline())
    BER = []
    FER = []
    ENc = []
    queue1 = Queue()
    queue2 = Queue()
    queue3 = Queue()
    queue4 = Queue()
    queue5 = Queue()
    queue6 = Queue()
    queue7 = Queue()
    queue8 = Queue()
    queue9 = Queue()
    queue10 = Queue()
    queue11 = Queue()
    queue12 = Queue()
    queue13 = Queue()
    queue14 = Queue()
    queue15 = Queue()
    queue16 = Queue()

    for snr in range(Numsnr):
        frame = 0
        frameerr = 0
        accerr = 0
        Ne = 0
        line1 = f.readline()
        while line1[0] != '!':
            line2 = f.readline()
            line3 = f.readline()
            if line3[0] == '!':
                break
            line4 = f.readline()
            line5 = f.readline()
            if line5[0] == '!':
                break
            line6 = f.readline()
            line7 = f.readline()
            if line7[0] == '!':
                break
            line8 = f.readline()
            line9 = f.readline()
            if line9[0] == '!':
                break
            line10 = f.readline()
            line11 = f.readline()
            if line11[0] == '!':
                break
            line12 = f.readline()
            line13 = f.readline()
            if line13[0] == '!':
                break
            line14 = f.readline()
            line15 = f.readline()
            if line15[0] == '!':
                break
            line16 = f.readline()
            line17 = f.readline()
            if line17[0] == '!':
                break
            line18 = f.readline()
            line19 = f.readline()
            if line19[0] == '!':
                break
            line20 = f.readline()
            line21 = f.readline()
            if line21[0] == '!':
                break
            line22 = f.readline()
            line23 = f.readline()
            if line23[0] == '!':
                break
            line24 = f.readline()
            line25 = f.readline()
            if line25[0] == '!':
                break
            line26 = f.readline()
            line27 = f.readline()
            if line27[0] == '!':
                break
            line28 = f.readline()
            line29 = f.readline()
            if line29[0] == '!':
                break
            line30 = f.readline()
            line31 = f.readline()
            if line31[0] == '!':
                break
            line32 = f.readline()
            frame = frame + 16
            y = list(map(eval, line2.split()))
            p1 = Process(target=thr1, args=(y, line1, N, K, n_var, a, We, queue1, index))
            y = list(map(eval, line4.split()))
            p2 = Process(target=thr2, args=(y, line3, N, K, n_var, a, We, queue2, index))
            y = list(map(eval, line6.split()))
            p3 = Process(target=thr3, args=(y, line5, N, K, n_var, a, We, queue3, index))
            y = list(map(eval, line8.split()))
            p4 = Process(target=thr4, args=(y, line7, N, K, n_var, a, We, queue4, index))
            y = list(map(eval, line10.split()))
            p5 = Process(target=thr5, args=(y, line9, N, K, n_var, a, We, queue5, index))
            y = list(map(eval, line12.split()))
            p6 = Process(target=thr6, args=(y, line11, N, K, n_var, a, We, queue6, index))
            y = list(map(eval, line14.split()))
            p7 = Process(target=thr7, args=(y, line13, N, K, n_var, a, We, queue7, index))
            y = list(map(eval, line16.split()))
            p8 = Process(target=thr8, args=(y, line15, N, K, n_var, a, We, queue8, index))
            y = list(map(eval, line18.split()))
            p9 = Process(target=thr9, args=(y, line17, N, K, n_var, a, We, queue9, index))
            y = list(map(eval, line20.split()))
            p10 = Process(target=thr10, args=(y, line19, N, K, n_var, a, We, queue10, index))
            y = list(map(eval, line22.split()))
            p11 = Process(target=thr11, args=(y, line21, N, K, n_var, a, We, queue11, index))
            y = list(map(eval, line24.split()))
            p12 = Process(target=thr12, args=(y, line23, N, K, n_var, a, We, queue12, index))
            y = list(map(eval, line26.split()))
            p13 = Process(target=thr13, args=(y, line25, N, K, n_var, a, We, queue13, index))
            y = list(map(eval, line28.split()))
            p14 = Process(target=thr14, args=(y, line27, N, K, n_var, a, We, queue14, index))
            y = list(map(eval, line30.split()))
            p15 = Process(target=thr15, args=(y, line29, N, K, n_var, a, We, queue15, index))
            y = list(map(eval, line32.split()))
            p16 = Process(target=thr16, args=(y, line31, N, K, n_var, a, We, queue16, index))

            p1.start()
            p2.start()
            p3.start()
            p4.start()
            p5.start()
            p6.start()
            p7.start()
            p8.start()
            p9.start()
            p10.start()
            p11.start()
            p12.start()
            p13.start()
            p14.start()
            p15.start()
            p16.start()
            r1 = queue1.get()
            r2 = queue2.get()
            r3 = queue3.get()
            r4 = queue4.get()
            r5 = queue5.get()
            r6 = queue6.get()
            r7 = queue7.get()
            r8 = queue8.get()
            r9 = queue9.get()
            r10 = queue10.get()
            r11 = queue11.get()
            r12 = queue12.get()
            r13 = queue13.get()
            r14 = queue14.get()
            r15 = queue15.get()
            r16 = queue16.get()

            if r1[0] != 0:
                frameerr = frameerr + 1
            if r2[0] != 0:
                frameerr = frameerr + 1
            if r3[0] != 0:
                frameerr = frameerr + 1
            if r4[0] != 0:
                frameerr = frameerr + 1
            if r5[0] != 0:
                frameerr = frameerr + 1
            if r6[0] != 0:
                frameerr = frameerr + 1
            if r7[0] != 0:
                frameerr = frameerr + 1
            if r8[0] != 0:
                frameerr = frameerr + 1
            if r9[0] != 0:
                frameerr = frameerr + 1
            if r10[0] != 0:
                frameerr = frameerr + 1
            if r11[0] != 0:
                frameerr = frameerr + 1
            if r12[0] != 0:
                frameerr = frameerr + 1
            if r13[0] != 0:
                frameerr = frameerr + 1
            if r14[0] != 0:
                frameerr = frameerr + 1
            if r15[0] != 0:
                frameerr = frameerr + 1
            if r16[0] != 0:
                frameerr = frameerr + 1

            accerr = accerr + r1[0] + r2[0] + r3[0] + r4[0] + r5[0] + r6[0] + r7[0] + r8[0] + r9[0] + r10[0] + r11[0] + \
                     r12[0] + r13[0] + r14[0] + r15[0] + r16[0]
            # print(frameerr, accerr)
            # time.sleep(3)

            Ne = Ne + r1[1] + r2[1] + r3[1] + r4[1] + r5[1] + r6[1] + r7[1] + r8[1] + r9[1] + r10[1] + r11[1] + r12[1] + \
                 r13[1] + r14[1] + r15[1] + r16[1]

            line1 = f.readline()
        BER.append(accerr / frame / K)
        FER.append(frameerr / frame)
        ENc.append(Ne / frame)
        print("BER")
        print(snr + 1, BER[snr])
        print("FER")
        print(snr + 1, FER[snr])
        print("En constraints broken rate:")
        print(snr + 1, ENc[snr])

    f.close()
    print("BER")
    for snr in range(Numsnr):
        print(snr + 1, BER[snr])
    print("FER")
    for snr in range(Numsnr):
        print(snr + 1, FER[snr])
    print("En constraints broken rate:")
    for snr in range(Numsnr):
        print(snr + 1, ENc[snr])