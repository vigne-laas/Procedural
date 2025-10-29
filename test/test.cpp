#include <iostream>
#include <vector>
#include <map>
#include <unordered_set>

struct VariableTable_t
{
    std::map<std::string, std::string> variables;
    std::unordered_set<std::string> agents;
};

class Fact
{
public:
    Fact(const std::string& s, const std::string& p, const std::string& o) : subject_(s), predicate_(p), object_(o) {}
    const std::string& getSubject() const { return subject_; }
    const std::string& getPredicate() const { return predicate_; }
    const std::string& getObject() const { return object_; }

    std::string toString() const
    {
        return subject_ + " " + predicate_ + " " + object_;
    }

    bool match(const Fact& other) const
    {
        return subject_ == other.subject_ && predicate_ == other.predicate_ && object_ == other.object_;
    }

private:
    std::string subject_;
    std::string predicate_;
    std::string object_;
};

class Observation
{
public:
    explicit Observation(int64_t id) : id_(id) {}
    virtual ~Observation() {}
    virtual std::pair<int64_t, VariableTable_t> getData()
    {
        return {id_, variables_};
    }

    virtual bool operator==(const Observation& other) const
    {
        return id_ == other.id_;
    }

    int64_t getId() const { return id_; }

protected:
    int64_t id_;
    VariableTable_t variables_;
};

class ObservationFact : public Observation
{
public:
    ObservationFact(const Fact& fact) : Observation((0x01<<64) | (int64_t)std::hash<std::string>{}(fact.getPredicate())),
                                        fact_(fact)
    {}

    bool operator==(const Observation& other) const override
    {
        if(id_ == other.getId())
        {
            const ObservationFact* other_fact = dynamic_cast<const ObservationFact*>(&other);
            if(other_fact)
            {
                return fact_.match(other_fact->fact_);
            }
        }
    }

private:
    Fact fact_;
};

class Transition
{
public:
    Transition() {}
    virtual ~Transition() {}
    virtual void match(Observation* observation) = 0;
private:
    Observation* observation_;


};








int main()
{

    return 0;
}