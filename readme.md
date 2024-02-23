**SRcdFuzzy** is a software developed in MATLAB for simulating the adaptive fuzzy iterative learning controller (AF-ILC) and the adaptive fuzzy repetitive generalized predictive controller (AFR-GPC). These controllers, proposed in a companion paper by Pereira et al., 2023, aim to minimize cyclical disturbances with small frequency range variations common in industrial process control loops. They employ fuzzy logic to estimate the disturbance cycle, using rules based on previous values of the integral of the absolute error between the setpoint and output. This estimated disturbance cycle period is then communicated to the regulatory controllers, guiding specific control actions to dampen oscillations in the process output. Additionally, each controller features a dedicated interface, providing testing options under various scenarios, including disturbances with different frequencies.

Software History
---

The code was used to the results reported in the following work:

Pereira, R.P. do A., Andrade E.J.F., Salles, J.L.F., Valadão, C. T. , Monteiro, R.S.M., de Almeida G.M., Cuadros, M.A.S.L., Bastos-Filho, T.F. Self-tuning regulatory controller of cyclical disturbances using data-driven frequency estimator based on fuzzy logic. Eng. Application of Artificial Intelligence. 126, 106987. [https://doi.org/10.1016/j.engappai.2023.106987](https://doi.org/10.1016/j.engappai.2023.106987)

Pre-requisite
---

It is necessary to have the MATLAB 2018b or higher installed.

Installation/use
---

**AF-ILC SIMULATION**

Inside the Github repository "Fuzzy", owned by user "RogerioPAP", there is a folder named AF-ILC, which contains the codes responsible for simulating the AF-ILC controller applied to a mold plant. In this folder, the main code "AFILC_main.m" manages all the algorithms. It calls the scripts: "moldebulg.m", which has the model of the mold plant; "update_ILC_Var_N.m", which updates the ILC matrix; and the script "interface.m", used to generate the graphical user interface. Moreover, the main code initiates the reading of the file "Fuzzy_ilc.fis," which contains the Fuzzy Rules, and also executes the routine responsible for creating the cyclical disturbance.


**AFR-GPC SIMULATION**

Inside the Github repository "Fuzzy", owned by the user "RogerioPAP", there is a folder named AFR-GPC, which contains the codes responsible for simulating the AFR-GPC controller applied to a mold plant. The algorithm of this controller, shown in Figure 3 of Pereira et al., 2023, combines the GPC in the direct loop and R-GPC with an N-period internal model in the feedback. In this folder, the main code "AFR_GPC_main.m" manages all the algorithms. It calls the scripts: "diophantine.m", responsible for solving the Diophantine Equation of GPC; "moldebulg.m", which has the model of the mold; and the script "interface.m", used to generate the graphical user interface. Furthermore, the main code calls for reading the file "AFR-GPC.fis," which contains the Fuzzy Rules, and executes the routine responsible for creating the cyclical disturbance.

** How to use **

 First run the file "interface.m" in MATLAB, associated with the desired application (AF-ILC or AFR-GPC),that is, run the " interface.m file from the AF-ILC directory to run the AF-ILC simulator and run the " interface.m file from the AFR-GPC directory to run the AFR-GPC simulator.
 Following this, choose the desired scenario and click on the "Start" button. The scenarios are better detailed in Pereira et al., 2023. The following Figures (a) and (b) show the user interfaces for AF-ILC and AFR-GPC.

![](RackMultipart20240131-1-kk0k4j_html_d33aa4766c622c8.png)

Authors
---

- Rogério P. Pereira
- Eduardo J. F. Andrade
- José L. F. Salles
- Carlos T. Valadão
- Ravena S. Monteiro
- Gustavo Maia de Almeida
- Marco A. S. L. Cuadros
- Teodiano F. Bastos-Filho

Citing SRcdFuzzy
---

We kindly ask users to cite the following reference in any publications reporting work done with SRcdFuzzy:

```bibtex
@article{PEREIRA2023106987,
title = {Self-tuning regulatory controller of cyclical disturbances using data-driven frequency estimator based on fuzzy logic},
journal = {Engineering Applications of Artificial Intelligence},
volume = {126},
pages = {106987},
year = {2023},
issn = {0952-1976},
doi = {https://doi.org/10.1016/j.engappai.2023.106987},
url = {https://www.sciencedirect.com/science/article/pii/S0952197623011715},
author = {Rogério P. Pereira and Eduardo J.F. Andrade and José L.F. Salles and Carlos T. Valadão and Ravena S. Monteiro and Gustavo Maia {de Almeida} and Marco A.S.L. Cuadros and Teodiano F. Bastos-Filho}
}
```

License
---

SRcdFuzzy is released under the MIT license. See the LICENSE file for details. All new contributions must be made under the MIT license.
![MIT License](mit_license_red.jpg)


Institutional support
---
![IFES-UFES](ifes-ufes.jpg)


Funding
---
![CAPES-FAPES-CNPq](capes-fapes-cnpq.jpg)