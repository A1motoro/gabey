# 参考文献清单（**已决定恢复引用**，编号 [1]–[10]）

> **编号与理由的权威表**：[section2_zh.md](section2_zh.md) §A（每条编号"要一篇什么文献、为什么非引不可、候选是谁"）。本文件只补核实状态与操作细节，**编号以 §A 为准**。
> **使用前必须逐条人工核实**（作者/年份/题名/出处四项），核实不了就按 [section2_zh.md](section2_zh.md) §C 的替换句式删掉该引用，不要硬用。
> 依据：`project_guidelines.md:76` — `Misuse of AI tools — such as submitting unverified AI-generated work as one's own, fabricating results, or using AI to cheat — is academic dishonesty.`

## 与 §2 四段对应的候选（按段落分组，不是按字母序）

| 对应段落 | 候选文献 | 核实状态 |
| --- | --- | --- |
| P1 风险与 AEB | `ISO 15623:2013. Intelligent transport systems — Forward vehicle collision warning systems — Performance requirements and test procedures.` | 存在性已确认；版本号待核 |
| P1 | Euro NCAP AEB test protocol — **版本年份待核**（不同年版本不同，写错不如不写） | 待核 |
| P1 | Shalev-Shwartz, S., Shammah, S., Shashua, A. (2017). *On a Formal Model of Safe and Scalable Self-driving Cars.* arXiv:1708.06374. （RSS） | 待核 |
| P2 运动预测 | Lefèvre, S., Vasquez, D., Laugier, C. (2014). *A survey on motion prediction and risk assessment for intelligent vehicles.* ROBOMECH Journal 1:1. | 已确认存在 |
| P2 | Schwarting, W., Alonso-Mora, J., Rus, D. (2018). *Planning and Decision-Making for Autonomous Vehicles.* Annual Review of Control, Robotics, and Autonomous Systems 1:187–210. | 待核 |
| P2 | Schubert, R., Mattern, N., Wöhler, C., Wanielik, G. (2011). *Empirical evaluation of vehicular models for ego motion estimation.* IEEE Intelligent Vehicles Symposium (IV), Baden-Baden. （CTRV） | 会议与作者已确认；题名/页码待核 |
| P2 | Alahi, A. et al. (2016). *Social LSTM: Human Trajectory Prediction in Crowded Spaces.* CVPR. | 待核 |
| P2 | Salzmann, T. et al. (2020). *Trajectron++: Dynamically-Feasible Trajectory Forecasting with Heterogeneous Data.* ECCV. | 待核 |
| P2 | Hogan, N. (1984). *An organizing principle for a class of voluntary movements.* Journal of Neuroscience 4(11):2745–2754. （最小抖动 / quintic） | 待核 |
| P3 数据 | Caesar, H. et al. (2020). *nuScenes: A Multimodal Dataset for Autonomous Driving.* CVPR. | 待核 |
| P3 | Wilson, B. et al. (2021). *Argoverse 2: Next Generation Datasets for Self-Driving Perception and Forecasting.* NeurIPS Datasets and Benchmarks. | 待核 |
| P3 | Krajewski, R. et al. (2018). *The highD Dataset: A Drone Dataset of Naturalistic Vehicle Trajectories on German Highways.* IEEE ITSC. | 待核 |
| P3 | Zhan, W. et al. (2019). *INTERACTION Dataset: An INTERnational, Adversarial and Cooperative moTION Dataset in Interactive Driving Scenarios with Semantic Maps.* arXiv / ICCV workshop. | 待核 |
| §1 备用 | Paden, B., Čáp, M., Yong, S. Z., Yershov, D., Frazzoli, E. (2016). *A Survey of Motion Planning and Control Techniques for Self-Driving Urban Vehicles.* IEEE Transactions on Intelligent Vehicles 1(1). | 待核；**仅当 §1 引用时才用**，否则删除（列而未引会减分） |

## 每条文献的网址（2026-10-25 逐条打开核对过）

> 编号同 [section2_zh.md](section2_zh.md) §A。**状态**栏：`已核实` = 我打开过页面、题名/出处一致；`待核` = 链接有效但页面内容（页码、题名细节）需你亲自看一眼。

| # | 文献 | 网址（主） | 备用 / DOI | 打开后确认什么 | 状态 |
| --- | --- | --- | --- | --- | --- |
| [1] | ISO 15623:2013 | [NSAI 商品页（含题名/年份/页数）](https://shop.standards.ie/en-ie/standards/iso-15623-2013-579991_saig_iso_iso_1327511/) | [ANSI 预览 PDF](https://webstore.ansi.org/preview-pages/ISO/preview_ISO+15623-2013.pdf) | 标准号 15623、年份 2013、题名逐字；**ISO 官网 `iso.org` 有反爬（实测 403）**，不要以打不开为理由放弃 | 待核 |
| [2] | Euro NCAP AEB 测试协议 | [Euro NCAP Protocols 总页](https://www.euroncap.com/protocols/) | [Safety Assist 版协议 PDF](https://www.euroncap.com/media/80154/euro-ncap-assessment-protocol-sa-collision-avoidance-v1041.pdf) | 2026–2028 版把 AEB 归入 **Crash avoidance** 支柱；引用必须写版本年份 | 版本待定 |
| [3] | RSS（Shalev-Shwartz et al., 2017） | [arXiv:1708.06374](https://arxiv.org/abs/1708.06374) | [DOI](https://doi.org/10.48550/arXiv.1708.06374) | 题名 `On a Formal Model of Safe and Scalable Self-driving Cars` | 已核实 |
| [4] | Lefèvre et al. (2014) | [DOI 10.1186/s40648-014-0001-z](https://doi.org/10.1186/s40648-014-0001-z) | [ROBOMECH Journal 文章页](https://robomechjournal.springeropen.com/articles/10.1186/s40648-014-0001-z) | 期刊名、卷期（1:1）、年份 2014 | 已核实（DOI 可解析） |
| [5] | Schubert et al. (2011) | [IEEE Xplore 5940526](https://ieeexplore.ieee.org/abstract/document/5940526) | [DBLP（Gerd Wanielik 条目）](https://dblp.org/pid/39/4464.html?view=bibtex) | 会议 = IEEE IV 2011, Baden-Baden；题名与页码逐字 | 待核 |
| [6] | Social LSTM（Alahi et al., 2016） | [CVPR 2016 官方开放页](https://openaccess.thecvf.com/content_cvpr_2016/html/Alahi_Social_LSTM_Human_CVPR_2016_paper.html) | [PDF](https://openaccess.thecvf.com/content_cvpr_2016/papers/Alahi_Social_LSTM_Human_CVPR_2016_paper.pdf) | 页码 961–971；作者含 Alahi / Goel / Ramanathan / Robicquet / Fei-Fei / Savarese | 已核实 |
| [7] | Trajectron++（Salzmann et al., 2020） | [arXiv:2001.03093](https://arxiv.org/abs/2001.03093) | [代码](https://github.com/StanfordASL/Trajectron-plus-plus) | arXiv 注明 ECCV 2020 | 已核实 |
| [8] | Hogan (1984) 最小抖动 | [DOI 10.1523/JNEUROSCI.04-11-02745.1984](https://doi.org/10.1523/JNEUROSCI.04-11-02745.1984) | [PubMed 6502203](https://pubmed.ncbi.nlm.nih.gov/6502203/) | J. Neuroscience 4(11):2745–2754，1984-11 | 待核（页码已从检索结果确认） |
| [9] | nuScenes（Caesar et al., 2020） | [arXiv:1903.11027](https://arxiv.org/abs/1903.11027) | [官网](https://www.nuscenes.org/nuscenes) | arXiv 注明 `CVPR 2020 camera ready` | 已核实 |
| [10] | highD（Krajewski et al., 2018） | [IEEE Xplore 8569552](https://ieeexplore.ieee.org/abstract/document/8569552) | [数据集官网](https://www.highd-dataset.com/)（**已跳转到 levelxdata.com**） | 会议 = IEEE ITSC 2018；题名逐字 | 待核 |
| [11] | Argoverse 2（Wilson et al., 2021） | [arXiv:2301.00493](https://arxiv.org/abs/2301.00493) | [NeurIPS D&B 论文 PDF](https://datasets-benchmarks-proceedings.neurips.cc/paper_files/paper/2021/file/4734ba6f3de83d861c3176a6273cac6d-Paper-round2.pdf) | arXiv 注明 NeurIPS Datasets & Benchmarks | 已核实 |
| [12] | INTERACTION（Zhan et al., 2019） | [arXiv:1910.03088](https://arxiv.org/abs/1910.03088) | [数据集官网](https://interaction-dataset.com) | 题名含 `INTERnational, Adversarial and Cooperative moTION`；会议版本（ICCV Workshop）待核 | 已核实（arXiv） |
| [13] | Schwarting et al. (2018) | [DBLP 条目](https://dblp.org/rec/journals/arcras/SchwartingAR18.html) | [DOI 10.1146/annurev-control-060117-105157](https://doi.org/10.1146/annurev-control-060117-105157) | Annual Review of Control, Robotics, and Autonomous Systems, 2018 | 待核 |
| [14] | WOMD（Ettinger et al., 2021） | [arXiv:2104.10133](https://arxiv.org/abs/2104.10133) | [官方代码](https://github.com/waymo-research/waymo-open-dataset) | 题名 `Large Scale Interactive Motion Forecasting for Autonomous Driving: The Waymo Open Motion Dataset`；这是 motion 数据论文，不是 2019 年的感知数据集论文 | 2026-11-23 自动核实 arXiv citation 元数据的题名、作者与 2021 年份；人工复核待 A |

### 核实过程中踩到的两个坑（提醒你）

1. **不要凭记忆写 arXiv 编号。** 我原以为 Social LSTM 是 `arXiv:1607.03663`，实测那是 CMS 实验的喷注能量论文；已改用 CVPR 官方开放页。你们自己核实时请从题名搜起，不要从编号推题名。
2. **`iso.org` 直连会 403**（Cloudflare）。这不代表标准不存在，换 NSAI / ANSI 页面核即可；正式提交时引文写标准号，不必给 URL。

## 核实方法（每人 3 分钟一条）

1. 打开出版商页面 / arXiv 摘要页 / DBLP 条目；
2. 比对**作者、年份、题名、出处**四项，逐字看，不要凭印象；
3. 把上表的状态改成"已核实 YYYY-MM-DD（姓名）"；
4. 在 [ai_disclosure.md](ai_disclosure.md) 记一行"谁核实了哪几条"。

## 如果之后要加回引用

- 只在 [proposal.md](proposal.md) 的 §2 末尾新增 `## References` 一节，**不要**在别的文件另建一份；
- 正文用作者-年份（如 `(Lefèvre et al., 2014)`），与参考文献表的字母序对应，避免编号错位；
- 先核实再插入，不要先插入后核实。
