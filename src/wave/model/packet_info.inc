
 class MyTag : public Tag
 {
 public:
  /**
   * \brief Get the type ID.
   * \return the object TypeId
   */
  static TypeId GetTypeId (void);
  virtual TypeId GetInstanceTypeId (void) const;
  virtual uint32_t GetSerializedSize (void) const;
  virtual void Serialize (TagBuffer i) const;
  virtual void Deserialize (TagBuffer i);
  virtual void Print (std::ostream &os) const;

  // these are our accessors to our tag structure
  /**
   * Set the tag value
   * \param value The tag value.
   */
  void SetMsgId (uint32_t value);
  //void SetPrio (uint32_t value);
  void SetPrio (double value);
  void SetNodeId(uint32_t value);
  void SetEmitTime(uint64_t value);

  /**
   * Get the tag value
   * \return the tag value.
   */
  uint32_t GetMsgId (void) const;
  //uint32_t GetPrio (void) const;
  double GetPrio (void) const;
  uint32_t GetNodeId (void) const;
  uint64_t GetEmitTime (void) const;

 private:
  uint32_t m_MsgId;  //!< tag value
  //uint32_t m_Prio;  //!< tag value
  double m_Prio;  //!< tag value
  uint32_t m_NodeId;  //!< tag value
  uint64_t m_EmitTime;  //!< tag value
 };


 TypeId
 MyTag::GetTypeId (void)
 {
   static TypeId tid = TypeId ("ns3::MyTag")
     .SetParent<Tag> ()
     .AddConstructor<MyTag> ()
     .AddAttribute ("MsgId",
                    "Message Id",
                    EmptyAttributeValue (),
                    MakeUintegerAccessor (&MyTag::GetMsgId),
                    MakeUintegerChecker<uint32_t> ())
     .AddAttribute ("Prio",
                    "Priority",
                    EmptyAttributeValue (),
                    //MakeUintegerAccessor (&MyTag::GetPrio),
                    //MakeUintegerChecker<uint32_t> ())
                    MakeDoubleAccessor (&MyTag::GetPrio),
                    MakeDoubleChecker<double> ())
     .AddAttribute ("NodeId",
                    "Node Id",
                    EmptyAttributeValue (),
                    MakeUintegerAccessor (&MyTag::GetNodeId),
                    MakeUintegerChecker<uint32_t> ())
     .AddAttribute ("EmitTime",
                    "EmitTime",
                    EmptyAttributeValue (),
                    MakeUintegerAccessor (&MyTag::GetEmitTime),
                    MakeUintegerChecker<uint64_t> ())
   ;
   return tid;
 }

 TypeId
 MyTag::GetInstanceTypeId (void) const
 {
   return GetTypeId ();
 }

 uint32_t
 MyTag::GetSerializedSize (void) const
 {
   return (4*2)+8+8;
 }

 void
 MyTag::Serialize (TagBuffer i) const
 {
   i.WriteU32(m_MsgId);
   i.WriteDouble(m_Prio);
   i.WriteU32(m_NodeId);
   i.WriteU64(m_EmitTime);

 }

 void
 MyTag::Deserialize (TagBuffer i)
 {
   m_MsgId = i.ReadU32();
   m_Prio = i.ReadDouble();
   m_NodeId = i.ReadU32();
   m_EmitTime = i.ReadU64();
 }

 void
 MyTag::Print (std::ostream &os) const
 {
   os << "MsgId=" << (uint32_t)m_MsgId;
 }

 void
 MyTag::SetMsgId (uint32_t value)
 {
   m_MsgId = value;
 }

 uint32_t
 MyTag::GetMsgId (void) const
 {
   return m_MsgId;
 }

 /*void
 MyTag::SetPrio (uint32_t value)
 {
   m_Prio = value;
 }*/
 void
 MyTag::SetPrio (double value)
 {
   m_Prio = value;
 }

 /*uint32_t
 MyTag::GetPrio (void) const
 {
   return m_Prio;
 }*/
 double
 MyTag::GetPrio (void) const
 {
   return m_Prio;
 }
 void
 MyTag::SetNodeId(uint32_t value)
 {
   m_NodeId = value;
 }

 uint32_t
 MyTag::GetNodeId (void) const
 {
   return m_NodeId;
 }


 void
 MyTag::SetEmitTime(uint64_t value)
 {
   m_EmitTime = value;
 }

 uint64_t
 MyTag::GetEmitTime (void) const
 {
   return m_EmitTime;
 }
