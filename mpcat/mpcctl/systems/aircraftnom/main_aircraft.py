import muaompc 
mpc = muaompc.ltidt.setup_mpc_problem('sys_aircraft', verbose=True)
mpc.generate_c_files(numeric='float32')

