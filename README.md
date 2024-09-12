**Prove คำตอบข้อสอบข้อ 3**
(ถูกทั้ง q1, q2 และ q3)

เริ่มจากการหา Transformation matrix ด้วย Forward Kinematic
![S__7290883](https://github.com/user-attachments/assets/5ad07b57-b668-47d8-9045-c3ef55b1c9d8)

**เขียนโปรแกรม Prove ด้วย Matlab**
1) Define Transformation matrix ด้วย Forward Kinematic (จากที่คำนวณตามรูปข้างบน)
2) เขียนสมการหา q1_sol, q2_sol, q3_sol จากข้อสอบลงโปรแกรม
3) สร้าง Test case แบบสุ่มๆ 5 ชุด (กำหนดค่า q1, q2, q3, L1, L2, L3, L4, L5)
4) ใช้หลักการ prove แบบที่ 1 ของอาจารย์นุก

   4.1) เอาค่า q1, q2, q3 ที่กำหนดมา(แบบสุ่มๆ)ของแต่ละ Test case มาเข้าสมการ Forward Kinematic จะได้ค่า Px, Py, Pz

   4.2) เอาค่า Px, Py, Pz จากสมการ Forward Kinematic ในขั้นตอนก่อนหน้ามาคำนวณหา q1_sol, q2_sol, q3_sol จากสมการ Inverse Kinematic

   4.3) นำคำตอบ q1_sol, q2_sol, q3_sol (4 รูปแบบ) กลับไปเช็คด้วยสมการ Forward Kinematic เพื่อหา Px_sol, Py_sol, Pz_sol เทียบกับค่า Px, Py, Pz จากขั้นตอน 4.1

เมื่อตรวจสอบแล้ว Px_sol, Py_sol, Pz_sol (จากขั้นตอน 4.3) กับค่า Px, Py, Pz (จากขั้นตอน 4.1) ตรงกันทั้งหมด
จึง Prove ได้ว่าสมการหา q1_sol, q2_sol และ q3_sol จาก Inverse Kinematic ที่คิดได้ในข้อสอบถูกต้อง

Code ทั้งหมดอยู่ในไฟล์ prove_exam3.m
Output ทัั้ง 5 test case ที่ได้จาก code ใน matlab อยู่ในไฟล์ code_output.pdf
