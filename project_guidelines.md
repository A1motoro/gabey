# AIE1903 Course Project Guidelines

**AI Exploration III — AI for Autonomous Driving: A Hands-on Introduction**

**2026–2027 Fall Term**

**Who**? – Team formation

* Form groups of **4–6 students** voluntarily as soon as possible, and no later than 23:59:59, **September 27th, 2026** (Sunday, Week 3).
* If you are unable to find a team, or your team is unable to find enough members, we will help you complete it.
* Each team should designate a **representative** to communicate team information, submit project materials, and schedule appointments (e.g., project consultation, demonstrations) with the TA and the instructor through email.
	* While any team member is welcome to contact the TA and the instructor directly with their concerns, do not submit duplicate copies of materials (proposals, reports, code, etc.) from the same team; doing so will result in a deduction of points from your project score.

**What**? – Project contents and deliverables

Select a topic related to **AI for autonomous driving** by yourselves (see the project topic list distributed in Week 1 for suggested directions, including but not limited to: camera-based perception, BEV/3D visualization, HD-map exploration, trajectory prediction, route planning, obstacle avoidance, closed-loop simulation, driving-scene reasoning with vision-language models, safety/failure-case analysis, or comparison of system designs) and develop the project collaboratively.

You are strongly encouraged to work with real-world autonomous driving data — using small, accessible subsets rather than full large-scale datasets, so that everything runs on an ordinary computer. GPUs are provided for you but not required.

Different groups may choose different or similar directions; there is no direct competition among groups.

Submit the following materials by the deadlines specified below:

1. **Midterm proposal presentation**: a short in-class presentation (approximately **10 minutes** per group, plus **5 minutes** for Q&A and instructor feedback), presenting the proposal below. Each group must state its intended final output and its main data/tool/schedule risks. Feedback received should be incorporated into the final submitted proposal document.
2. **Project proposal**: a document (4-6 pages, about one page per member) including the following sections to answer the corresponding questions:
   1. **Introduction**: What topic have you chosen and why? Which part(s) of the autonomous-driving pipeline does it relate to?
   2. **Related work**: Where do your ideas come from, and where will you obtain additional resources (references, libraries, datasets, pre-trained models, etc.)?
   3. **Our work**: What will you implement in detail, e.g., the overall architecture and individual components in your project, and what are the highlights that make your project distinctive? Define a **minimum viable version**, an optional **extension**, and a **fallback plan** if tools or data turn out to be harder than expected.
   4. **Evidence and evaluation**: What will your final demo or analysis show? Complete the sentence: "Our final evidence will be …", and state what that evidence will and will not demonstrate.
   5. **Schedule**: What are the project milestones (at least bi-weekly) and their respective completion dates?
   6. **Team**: Who are the team members, and how will all the work be distributed among them?
3. **Final presentation and demo**: an in-class presentation in Week 13. Each group has approximately **10 minutes** for the presentation/demo and **10 minutes** for Q&A, feedback, and discussion.
4. **Project code package** (the main milestone deliverable): the project output, e.g., source code and, if possible, a runnable demo. Include a short README and an environment file so the package can be reproduced.
   * Projects are expected to run live on an ordinary laptop in Python. If your project cannot be delivered as a standalone package, add saved outputs or a recorded demo video, and explain it during the in-class final presentation.
5. **Project report**: a *substantial extension* of the proposal (approximately 8-12 pages, about two pages per member, formatted like a regular conference paper), including the following sections to answer the corresponding questions:
   1. **Introduction**: What have you done, and why? In particular, is there any change from the proposal, and if so, why?
   2. **Related work**: Where did your ideas come from, and where did you obtain additional resources? What did you borrow from these resources? Again, note any changes from the proposal and explain the reasons.
   3. **Our work**: What have you accomplished and how did you achieve it? What specifically have you done beyond the resources you borrowed? Present your results with explicit **claims, evidence, and limitations**: what does the evidence demonstrate, and under which operating conditions does it apply?
   4. **Contributions**: What work did each member do, and how do you evaluate each member's contribution (in terms of workload weights, summing up to the team size), which will be used for grade weighting?
   5. **Reflections**: What difficulties did you encounter and how did you overcome them? What have you learned through this project, distinguishing between classroom learning and self-directed learning? What are the limitations and safety considerations of your project? (This section should include a team reflection on collaboration and individual reflections from each member.)
   6. **AI-tool usage statement**: Which AI tools did you use, for what purpose, and how did you verify the correctness of their outputs?

**When**? – Deadlines

* **Team formation**: Due by 23:59:59, September 27th, (Sunday, Week 3).
* **Midterm proposal presentation (DL1)**: In class, **October 27th, 2026** (Tuesday, Week 7).
* **Project proposal (DL2)**: Due by 23:59:59, **November 1st, 2026**, (Sunday, Week 7).
* **Final presentation and demo (DL3)**: In class, **December 8th, 2026** (Tuesday, Week 13).
* **Project code package (DL4)**: Due by 23:59:59, **December 13th, 2026**, (Sunday, Week 13).
* **Project report (DL4)**: Due by 23:59:59, **December 13th, 2026**, (Sunday, Week 13).
* Before **DL2**, feedback given at the proposal presentation will help you improve your final proposal. Similarly you can improve your final report before **DL4**, given the feedback received at the final presentation. Any complaints (e.g., the workload distribution) will also be resolved by the instructor during this time.
* All materials are submitted via the university learning platform **BlackBoard**.

**How**? – Grading criteria

1. **Midterm proposal presentation**: 15%
2. **Midterm proposal document**: 15%
3. **Final presentation and demo**: 15%
4. **Final report**: 15%
5. **Milestone deliverables** (project code package and demo materials): 40%
6. **Assessment scheme**: For any of the above submissions, the assessment will be as follows:
   1. **01% ~ 20%**: Submission of some material;
   2. **21% ~ 40%**: Readable documents (with missing information) and runnable code/demo;
   3. **41% ~ 60%**: Informative documents (with essential information) and working code/demo;
   4. **61% ~ 80%**: Complete documents (with detailed information) and functional results matching the proposal;
   5. **81% ~ 100%**: Insightful documents and creative, interesting projects, in addition to meeting the above criteria.
7. In addition to the ladder above, evaluation considers clarity, technical effort, correctness, creativity, presentation quality, teamwork, and understanding of limitations. Honest claims matched to evidence are valued more than broad but unsupported claims.
8. If the workload of a project is considered significantly lower than expected, its score will be reduced accordingly.
9. **Each student's final grade will be based on the overall project grade, adjusted according to their individual contribution (the sum of workload weights equals the number of team members).**
10. For example: If a 6-member team has submitted a proposal with 6 planned parts of work, but only completes 5, resulting in a complete, functional project, their initial overall score of 78% (per rule 6.4) will be discounted to 65% (per rule 8). Individual team member grades will be: 78%, 71.5%, 65%, 65%, 58.5%, 52%, based on their specified contribution weights (1.2, 1.1, 1, 1, 0.9, 0.8) (per rule 9).

**Academic Honesty and AI Tool Use**

* AI tools are allowed and encouraged throughout this project — for learning, brainstorming, coding, debugging, visualization, literature exploration, and writing assistance. Students remain responsible for the correctness, originality, interpretation, and quality of their own work.
* Each team's report must include an **AI-tool usage statement** (see *What*, item 5.6), and every member must be able to explain how AI tools were used and what their own intellectual contributions were.
* Misuse of AI tools — such as submitting unverified AI-generated work as one's own, fabricating results, or using AI to cheat — is academic dishonesty.
* Students may be required to explain their work to the TA or the instructor at any time during the semester. This may include, for instance, explaining a randomly selected code segment from their project. Failure to adequately explain one's own code, which may be indicative of plagiarism, can result in an immediate grade of "F".